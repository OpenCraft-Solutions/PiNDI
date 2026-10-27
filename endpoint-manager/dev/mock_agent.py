#!/usr/bin/env python3
"""Fake PiNDI devices for developing the manager without Raspberry Pis.

    python3 dev/mock_agent.py --count 3      -> devices at 127.0.0.1:9441, :9442, :9443 (login pindi / pindi)

Each speaks the agent's HTTP API (version 2) over HTTPS with its own self-signed certificate (made with the
openssl command line tool). Metrics wander, routing and updates behave roughly like a real device; reboot and
update take the device "offline" (503) for a few seconds.
"""
import argparse
import http.server
import json
import os
import random
import re
import ssl
import subprocess
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SPLASH = os.path.join(HERE, "..", "..", "agent", "Splash.jpg")
CAMERAS = [("STUDIO-A (CAM 1)", "10.0.0.21"), ("STUDIO-A (CAM 2)", "10.0.0.22"), ("STUDIO-B (CAM 1)", "10.0.0.31"),
           ("GFX-PC (Program)", "10.0.0.40"), ("GFX-PC (Lower thirds)", "10.0.0.40"), ("LAPTOP-7 (Screen)", "10.0.0.77")]


class Device:
    def __init__(self, n, port):
        self.lock = threading.Lock()
        self.n, self.port = n, port
        self.serial = "1000%04d" % (8000 + n)
        self.name = "Mock-%d" % n
        self.token = "mock-token-%d" % n
        self.version = "1.4.0"
        self.down_until = 0
        self.update = {"state": "", "log": "", "done_at": 0}
        self.temp, self.load, self.mem, self.rx, self.tx = 48.0 + n, 0.5, 30.0, 8e6, 1e6
        count = 2 if n % 2 else 1
        self.outputs = [{"n": i, "stream": "", "source_ip": "", "connector": "HDMI-A-%d" % i} for i in range(1, count + 1)]
        self.started = time.time()

    def own_sources(self):
        return [("%s (IN_%s)" % (self.name.upper(), self.serial), "127.0.0.1")]

    def inputs(self):
        """Input 1 is sent over NDI; odd-numbered devices also send input 2 as H.264 RTP multicast, even-numbered
        ones as H.264 SRT (listening on port 9000 + n: the mock devices share an address)."""
        ins = [{"slot": 1, "device": "/dev/v4l/by-id/usb-MOCK", "present": True, "enabled": True,
                "name": "IN_%s" % self.serial, "state": "ON AIR", "mode": "MJPG 1920x1080@30", "fps": "30.0",
                "audio": "48 kHz stereo", "on_air": True, "codec": "speedhq", "rtp_dest": "", "srt_url": ""}]
        two = {"slot": 2, "device": "/dev/v4l/by-id/usb-MOCK2", "present": True, "enabled": True,
               "name": "IN_%s_2" % self.serial, "mode": "MJPG 1920x1080@30", "fps": "30.0", "audio": "48 kHz stereo",
               "on_air": False, "rtp_dest": "", "srt_url": ""}
        if self.n % 2:
            dest = "239.255.0.%d:5004" % self.n
            ins.append(dict(two, state="sending H.264 RTP to %s (30 fps)" % dest, codec="rtp", rtp_dest=dest))
        else:
            port = 9000 + self.n
            ins.append(dict(two, state="sending H.264 SRT to 1 receiver on port %d (30 fps)" % port, codec="srt",
                            srt_url="srt://:%d" % port))
        return ins

    def tick(self):
        with self.lock:
            self.temp = min(85, max(35, self.temp + random.uniform(-0.8, 0.8)))
            self.load = max(0, self.load + random.uniform(-0.2, 0.2))
            self.mem = min(90, max(15, self.mem + random.uniform(-1, 1)))
            self.rx = max(0, self.rx + random.uniform(-2e6, 2e6))
            self.tx = max(0, self.tx + random.uniform(-5e5, 5e5))
            u = self.update
            if u["state"] == "running" and time.time() > u["done_at"]:
                u["state"], u["log"] = "done", u["log"] + "Update installed, rebooting.\n"
                self.version = u["new"]
                self.down_until = time.time() + 12
                self.started = self.down_until

    def status(self, all_sources):
        outs = []
        for o in self.outputs:
            target = o["stream"] or ("OUT_%s" % self.serial if o["n"] == 1 else "OUT_%s_%d" % (self.serial, o["n"]))
            if target.lower().startswith("rtp://"):  # an RTP output's source is the address it receives
                match = next(((target, "127.0.0.1") for d in DEVICES for i in d.inputs()
                              if i["codec"] == "rtp" and "rtp://" + i["rtp_dest"] == target), None)
                fmt = "H.264 RTP, hardware decoded"
            elif target.lower().startswith("srt://"):  # as RTP: the srt:// address it calls
                match = next(((target, "127.0.0.1") for d in DEVICES for i in d.inputs()
                              if i["codec"] == "srt" and target == "srt://127.0.0.1:%d" % (9000 + d.n)), None)
                fmt = "H.264 SRT, hardware decoded"
            else:
                match = next((s for s in all_sources if s[0].lower().endswith("(%s)" % target.lower()) and
                              (not o["source_ip"] or s[1] == o["source_ip"])), None)
                fmt = "Full NDI (SpeedHQ)"
            outs.append({"n": o["n"], "connector": o["connector"], "enabled": True,
                         "state": "playing" if match else "waiting for %s" % target, "target": target,
                         "source": match[0] if match else "", "video": "1920x1080 59.94p" if match else "",
                         "format": fmt if match else "", "audio": "", "latency": "2 frames" if match else "",
                         "stream": o["stream"], "source_ip": o["source_ip"], "backup": "",
                         "display": {"link": "connected", "mode": "1920x1080@60"}})
        up = int(time.time() - self.started)
        return {"api_version": 3,
                "device": {"name": self.name, "ndi_name": self.name.upper(), "serial": self.serial,
                           "model": "Raspberry Pi 4 Model B" if self.n % 2 else "Raspberry Pi 5", "version": self.version,
                           "ip": "127.0.0.1", "health": "CPU %.0f °C · load %.1f (4 cores)" % (self.temp, self.load),
                           "power": "OK" if self.temp < 80 else "Now: soft temperature limit.",
                           "network": "%.1f Mbit/s in, %.1f Mbit/s out" % (self.rx / 1e6, self.tx / 1e6),
                           "metrics": {"cpu_temp": round(self.temp, 1), "load": round(self.load, 2), "cores": 4,
                                       "memory_pct": round(self.mem, 1), "storage_pct": 41.0, "uptime": up,
                                       "throttled": 0, "rx_bps": int(self.rx), "tx_bps": int(self.tx), "link_mbps": 1000},
                           "update": self.update["state"]},
                "outputs": outs,
                "inputs": self.inputs(),
                "sources": [{"name": s[0], "stream": re.match(r"^.*? \((.*)\)$", s[0]).group(1), "ip": s[1]}
                            for s in all_sources]}


DEVICES = []


def all_sources():
    return CAMERAS + [s for d in DEVICES for s in d.own_sources()]


def handler_for(dev):
    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def send(self, code, body, ctype="application/json"):
            data = json.dumps(body).encode() if ctype == "application/json" else body
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def gate(self):
            if time.time() < dev.down_until:
                self.send(503, {"error": "rebooting"})
                return False
            if self.path == "/api/v1/token":
                return True
            if self.headers.get("Authorization", "") != "Bearer " + dev.token:
                self.send(401, {"error": "missing or wrong API token"})
                return False
            return True

        def body(self):
            return self.rfile.read(int(self.headers.get("Content-Length", "0") or 0))

        def do_GET(self):
            if not self.gate():
                return
            if self.path == "/api/v1/status":
                with dev.lock:
                    return self.send(200, dev.status(all_sources()))
            if self.path == "/api/v1/update":
                return self.send(200, {"state": dev.update["state"], "log": dev.update["log"]})
            if re.match(r"^/api/v1/outputs/\d/thumb\.jpg$", self.path) and os.path.exists(SPLASH):
                with open(SPLASH, "rb") as f:
                    return self.send(200, f.read(), "image/jpeg")
            self.send(404, {"error": "unknown endpoint"})

        def do_POST(self):
            if not self.gate():
                return
            raw = self.body()
            path = self.path.split("?")[0]
            if path == "/api/v1/token":
                data = json.loads(raw or b"{}")
                if data.get("user") == "pindi" and data.get("password") == "pindi":
                    return self.send(200, {"token": dev.token})
                time.sleep(1)
                return self.send(401, {"error": "Wrong user or password, or the user is not an administrator."})
            m = re.match(r"^/api/v1/outputs/(\d+)/source$", path)
            if m:
                data = json.loads(raw or b"{}")
                o = next((o for o in dev.outputs if o["n"] == int(m.group(1))), None)
                if not o:
                    return self.send(404, {"error": "unknown output"})
                with dev.lock:
                    o["stream"], o["source_ip"] = data.get("stream", ""), data.get("source_ip", "")
                return self.send(200, {"ok": True, "output": o["n"], "stream": o["stream"]})
            if path in ("/api/v1/identify", "/api/v1/restart"):
                return self.send(200, {"ok": True, "message": "done"})
            if path == "/api/v1/reboot":
                dev.down_until, dev.started = time.time() + 15, time.time() + 15
                return self.send(200, {"ok": True})
            if path == "/api/v1/shutdown":  # off until the mock is restarted
                dev.down_until = float("inf")
                return self.send(200, {"ok": True})
            if path == "/api/v1/update":
                if not raw.startswith(b"\x1f\x8b"):
                    return self.send(400, {"ok": False, "message": "That is not a valid update package."})
                with dev.lock:
                    new = "1.4.%d" % (int(dev.version.split(".")[-1]) + 1)
                    dev.update = {"state": "running", "new": new, "done_at": time.time() + 10,
                                  "log": "PiNDI update %s -> %s, %s\nInstalling...\n" % (dev.version, new, time.ctime())}
                return self.send(200, {"ok": True, "message": "Installing PiNDI %s." % new})
            self.send(404, {"error": "unknown endpoint"})
    return H


def make_cert(folder, n):
    cert, key = os.path.join(folder, "c%d.pem" % n), os.path.join(folder, "k%d.pem" % n)
    subprocess.run(["openssl", "req", "-x509", "-newkey", "ec", "-pkeyopt", "ec_paramgen_curve:prime256v1", "-nodes",
                    "-days", "30", "-subj", "/CN=mock-%d" % n, "-keyout", key, "-out", cert],
                   check=True, capture_output=True)
    return cert, key


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--count", type=int, default=3)
    ap.add_argument("--port", type=int, default=9441)
    args = ap.parse_args()
    tmp = tempfile.mkdtemp(prefix="pindi-mock-")
    for i in range(args.count):
        dev = Device(i + 1, args.port + i)
        DEVICES.append(dev)
        cert, key = make_cert(tmp, i)
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(cert, key)
        srv = http.server.ThreadingHTTPServer(("127.0.0.1", dev.port), handler_for(dev))
        srv.socket = ctx.wrap_socket(srv.socket, server_side=True)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        print("mock device %s at 127.0.0.1:%d" % (dev.name, dev.port), flush=True)
    while True:
        time.sleep(2)
        for d in DEVICES:
            d.tick()


if __name__ == "__main__":
    main()
