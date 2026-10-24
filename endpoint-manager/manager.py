#!/usr/bin/env python3
"""PiNDI Endpoint Manager: monitoring, video matrix, discovery, installs and bulk administration for PiNDI
devices.

Standard library only (Python 3.8+); installing PiNDI on a new Raspberry Pi over SSH also needs paramiko.
Talks to each device's HTTP API (agent 1.3.0 or newer for everything; older agents through their web
interface where the API lacks something).

    python3 manager.py                      -> http://127.0.0.1:8080/  (sign in: pindi / pindi)
    python3 manager.py --host 0.0.0.0 --cert cert.pem --key key.pem    -> HTTPS for the whole network

Devices are reached over HTTPS. Their certificates are usually self-signed, so each device's certificate
fingerprint is remembered when it is added (trust on first use) and checked before its API token is sent;
SSH host keys likewise. Saved data (a per-user folder, owner-only) never holds device passwords; API tokens
in it are encrypted with the Windows user's key (DPAPI) on Windows.
"""
import argparse
import base64
import collections
import concurrent.futures
import ctypes
import getpass
import glob
import hashlib
import hmac
import html
import http.client
import http.cookies
import http.server
import ipaddress
import json
import mimetypes
import os
import re
import secrets
import socket
import ssl
import subprocess
import sys
import threading
import time
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
VERSION = open(os.path.join(HERE, "VERSION")).read().strip()
STATIC_DIR = os.path.join(HERE, "static")

POLL_SEC = 5
HISTORY = 720  # points per device: one hour at POLL_SEC
DEVICE_TIMEOUT = 4
UPDATE_TIMEOUT = 300  # uploading a package to a slow device
UPDATE_MAX = 32 * 1024 * 1024  # the agent's own limit
BODY_MAX = 65536
SESSION_HOURS = 12
DEFAULT_USER, DEFAULT_PASSWORD = "pindi", "pindi"
PBKDF2_ROUNDS = 300000


def log(msg):
    print(time.strftime("%Y-%m-%d %H:%M:%S"), msg, flush=True)


def write_json(path, data):
    """Atomically, readable by the owner only (devices.json holds API tokens)."""
    tmp = path + ".tmp"
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, path)


def read_json(path, default):
    try:
        with open(path) as f:
            return json.load(f)
    except FileNotFoundError:
        return default


# ---------------------------------------------------------------- saved data: where, who can read it, secrets

def default_data_dir():
    """Per user, outside the repository (so it is never committed or copied along with the code)."""
    if os.name == "nt":
        return os.path.join(os.environ.get("LOCALAPPDATA") or os.path.expanduser("~"), "PiNDI Manager")
    return os.path.join(os.environ.get("XDG_CONFIG_HOME") or os.path.expanduser("~/.config"), "pindi-manager")


def lock_down(folder):
    """Only the current user may open the data folder (and what is made in it)."""
    if os.name != "nt":
        os.chmod(folder, 0o700)
        return
    # Windows ignores chmod: drop the inherited ACL, grant this user full control, and remove the broad groups
    # (Everyone, Authenticated Users, Users) should they be set on the folder itself. SYSTEM and Administrators
    # may keep access, as for Windows OpenSSH keys; DPAPI keeps the tokens unreadable to them anyway.
    user = getpass.getuser()
    if os.environ.get("USERDOMAIN"):
        user = os.environ["USERDOMAIN"] + "\\" + user
    # The folder only (no /T): its files inherit from it, so they follow without losing their own access.
    r = subprocess.run(["icacls", folder, "/inheritance:r", "/grant:r", "%s:(OI)(CI)F" % user,
                        "/remove:g", "*S-1-1-0", "*S-1-5-11", "*S-1-5-32-545", "/Q"],
                       capture_output=True, text=True)
    for name in os.listdir(folder):  # files an earlier version locked down one by one: inherit again
        subprocess.run(["icacls", os.path.join(folder, name), "/reset", "/Q"], capture_output=True)
    if r.returncode:
        log("warning: could not restrict %s to %s: %s" % (folder, user, (r.stderr or r.stdout).strip()))


class _Blob(ctypes.Structure):
    _fields_ = [("cbData", ctypes.c_uint32), ("pbData", ctypes.POINTER(ctypes.c_char))]


_ENTROPY = b"PiNDI Endpoint Manager"


def _dpapi(data, protect):
    """Windows DPAPI: data only this Windows user (on this computer) can decrypt."""
    crypt32, kernel32 = ctypes.windll.crypt32, ctypes.windll.kernel32
    kernel32.LocalFree.argtypes = [ctypes.c_void_p]
    buf, ent = ctypes.create_string_buffer(data, len(data)), ctypes.create_string_buffer(_ENTROPY, len(_ENTROPY))
    blob_in = _Blob(len(data), ctypes.cast(buf, ctypes.POINTER(ctypes.c_char)))
    entropy = _Blob(len(_ENTROPY), ctypes.cast(ent, ctypes.POINTER(ctypes.c_char)))
    blob_out = _Blob()
    fn = crypt32.CryptProtectData if protect else crypt32.CryptUnprotectData
    if not fn(ctypes.byref(blob_in), None, ctypes.byref(entropy), None, None, 0x1,  # CRYPTPROTECT_UI_FORBIDDEN
              ctypes.byref(blob_out)):
        raise OSError("DPAPI failed (error %d)" % ctypes.GetLastError())
    try:
        return ctypes.string_at(blob_out.pbData, blob_out.cbData)
    finally:
        kernel32.LocalFree(ctypes.cast(blob_out.pbData, ctypes.c_void_p))


def seal(text):
    """A secret as saved: DPAPI-encrypted on Windows; elsewhere the owner-only file is the protection."""
    if os.name == "nt":
        return "dpapi:" + base64.b64encode(_dpapi(text.encode(), True)).decode()
    return text


def unseal(text):
    if text.startswith("dpapi:"):
        return _dpapi(base64.b64decode(text[6:]), False).decode()
    return text


# ---------------------------------------------------------------- talking to devices

class DeviceError(Exception):
    def __init__(self, msg, reset=False):
        super().__init__(msg)
        self.reset = reset  # the device dropped the connection (older agents do when refusing a request body)


class CertChanged(DeviceError):
    def __init__(self, fingerprint):
        super().__init__("the device's certificate changed: check it, then trust the new one")
        self.fingerprint = fingerprint


# Certificates are checked by pinning (below), not by a CA: devices make their own.
NO_CA = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
NO_CA.check_hostname = False
NO_CA.verify_mode = ssl.CERT_NONE


def split_address(address):
    """"192.168.1.50" or "host:8443" -> (host, port)."""
    m = re.match(r"^\[?([A-Za-z0-9.:-]+?)\]?(?::(\d{1,5}))?$", address.strip())
    if not m:
        raise DeviceError("not a valid address: %s" % address)
    return m.group(1), int(m.group(2) or 443)


def fingerprint_of(der):
    return ":".join("%02X" % b for b in hashlib.sha256(der).digest())


def device_request(address, fingerprint, method, path, token=None, body=None, ctype="application/json",
                   timeout=DEVICE_TIMEOUT, headers=None):
    """(status, body bytes, response headers). The certificate is checked before anything is sent: with a
    fingerprint that no longer matches, CertChanged and no token leaves this computer."""
    host, port = split_address(address)
    conn = http.client.HTTPSConnection(host, port, context=NO_CA, timeout=timeout)
    try:
        conn.connect()
        seen = fingerprint_of(conn.sock.getpeercert(True))
        if fingerprint and not hmac.compare_digest(seen, fingerprint):
            raise CertChanged(seen)
        headers = dict(headers or {})
        if token:
            headers["Authorization"] = "Bearer " + token
        if body is not None:
            headers["Content-Type"] = ctype
            if isinstance(body, (dict, list)):
                body = json.dumps(body).encode()
        conn.request(method, path, body=body, headers=headers)
        resp = conn.getresponse()
        return resp.status, resp.read(), resp.headers
    except (OSError, http.client.HTTPException) as e:
        raise DeviceError(str(e) or e.__class__.__name__,
                          reset=isinstance(e, (ConnectionResetError, ConnectionAbortedError, BrokenPipeError)))
    finally:
        conn.close()


def device_json(address, fingerprint, method, path, token=None, body=None, **kw):
    status, data, _ = device_request(address, fingerprint, method, path, token, body, **kw)
    try:
        out = json.loads(data or b"{}")
    except ValueError:
        out = {}
    if status == 401:
        raise DeviceError(out.get("error") or "the device refused the API token")
    if status >= 400:
        raise DeviceError(out.get("error") or out.get("message") or "HTTP %d" % status)
    return out


def current_fingerprint(address):
    host, port = split_address(address)
    conn = http.client.HTTPSConnection(host, port, context=NO_CA, timeout=DEVICE_TIMEOUT)
    try:
        conn.connect()
        return fingerprint_of(conn.sock.getpeercert(True))
    except OSError as e:
        raise DeviceError("cannot reach %s over HTTPS: %s" % (address, e))
    finally:
        conn.close()


class WebLogin:
    """The device's own web interface, signed in like a browser. Agents older than 1.3.0 have no API for the
    token or for software updates, but their web interface does both."""

    def __init__(self, address, fingerprint, user, password):
        self.address, self.fingerprint, self.cookie = address, fingerprint, ""
        status, _, headers = self.request("POST", "/login", {"user": user, "password": password})
        m = re.search(r"pindi_session=([^;]+)", headers.get("Set-Cookie", ""))
        if status == 429:
            raise DeviceError("too many failed sign-ins on the device: wait 10 minutes")
        if not m:
            raise DeviceError("the device refused the login (wrong user or password, or not an administrator)")
        self.cookie = m.group(1)
        self.page = self.get("/")  # also takes the "default password" notice the sign-in left
        m = re.search(r'name="csrf" value="([^"]+)"', self.page)
        if not m:
            raise DeviceError("unexpected answer from the device's web interface")
        self.csrf = m.group(1)

    def request(self, method, path, form=None, timeout=DEVICE_TIMEOUT):
        body = urllib.parse.urlencode(form).encode() if form is not None else None
        return device_request(self.address, self.fingerprint, method, path, body=body,
                              ctype="application/x-www-form-urlencoded", timeout=timeout,
                              headers={"Cookie": "pindi_session=" + self.cookie} if self.cookie else None)

    def get(self, path):
        return self.request("GET", path)[1].decode("utf-8", "replace")

    def token(self):
        m = re.search(r"<dt>Token</dt><dd><code>([^<]+)</code>", self.page)
        if not m:
            raise DeviceError("the API token isn't on the device's page: paste it instead")
        return html.unescape(m.group(1))

    def update(self, package, downgrade):
        """Upload the package as the Software update form does; the device's own message on failure."""
        form = {"csrf": self.csrf, "package": "data:application/gzip;base64," + base64.b64encode(package).decode()}
        if downgrade:
            form["downgrade"] = "1"
        status, _, headers = self.request("POST", "/update", form, timeout=UPDATE_TIMEOUT)
        if status in (301, 302, 303) and headers.get("Location", "").endswith("/update"):
            return "Installing (through the device's web interface); it reboots when done."
        m = re.search(r'<div class="msg err">(.*?)</div>', self.get("/"))
        raise DeviceError(html.unescape(m.group(1)) if m else "the device did not accept the update")

    def close(self):
        try:
            self.request("POST", "/logout", {"csrf": self.csrf})
        except DeviceError:
            pass


def is_old_agent(err):
    """An agent older than 1.3.0 answering POST /api/v1/token: it wants a token (or drops the connection)."""
    return err.reset or "missing or wrong API token" in str(err) or "unknown endpoint" in str(err)


# ---------------------------------------------------------------- fleet

class Fleet:
    """The devices (devices.json) and what the poller last saw of each."""

    def __init__(self, path):
        self.path = path
        self.lock = threading.Lock()
        saved = read_json(path, [])
        self.devices = {d["id"]: dict(d, token=unseal(d["token"])) for d in saved}
        self.live = {}
        if any(seal("x") != "x" and not d["token"].startswith("dpapi:") for d in saved):
            self.save()  # tokens saved in plain text by an earlier version: encrypt them now

    def _live(self, dev_id):
        return self.live.setdefault(dev_id, {"online": False, "error": "not polled yet", "seen": None,
                                             "status": None, "cert_changed": None, "action": None,
                                             "history": collections.deque(maxlen=HISTORY)})

    def save(self):
        write_json(self.path, [dict(d, token=seal(d["token"])) for d in self.devices.values()])

    def addresses(self):
        with self.lock:
            return {split_address(d["address"])[0]: d["id"] for d in self.devices.values()}

    def get(self, dev_id):
        with self.lock:
            dev = self.devices.get(dev_id)
            if not dev:
                raise DeviceError("unknown device")
            return dict(dev)

    # --- polling

    def poll(self, dev):
        try:
            st = device_json(dev["address"], dev["fingerprint"], "GET", "/api/v1/status", dev["token"])
        except CertChanged as e:
            with self.lock:
                self._live(dev["id"]).update(online=False, error=str(e), cert_changed=e.fingerprint)
            return
        except DeviceError as e:
            with self.lock:
                self._live(dev["id"]).update(online=False, error=str(e))
            return
        with self.lock:
            self._record(dev["id"], st)
            name = (st.get("device") or {}).get("name")
            if name and self.devices.get(dev["id"], {}).get("name") != name:
                self.devices[dev["id"]]["name"] = name
                self.save()

    def _record(self, dev_id, st):
        """A status just read (lock held): the live state and a point for the graphs."""
        m = (st.get("device") or {}).get("metrics") or {}
        point = {"t": int(time.time())}
        for k in ("cpu_temp", "load", "memory_pct", "rx_bps", "tx_bps"):
            point[k] = m.get(k)
        live = self._live(dev_id)
        live.update(online=True, error="", seen=time.time(), status=st, cert_changed=None)
        live["history"].append(point)

    def poll_loop(self):
        with concurrent.futures.ThreadPoolExecutor(16) as pool:
            while True:
                started = time.monotonic()
                with self.lock:
                    devs = [dict(d) for d in self.devices.values()]
                list(pool.map(self.poll, devs))
                time.sleep(max(0.5, POLL_SEC - (time.monotonic() - started)))

    # --- state for the browser (never the tokens)

    def state(self):
        with self.lock:
            out = []
            for dev in sorted(self.devices.values(), key=lambda d: (d.get("name") or d["address"]).lower()):
                live = self._live(dev["id"])
                out.append({"id": dev["id"], "address": dev["address"], "name": dev.get("name") or dev["address"],
                            "serial": dev.get("serial", ""), "fingerprint": dev["fingerprint"],
                            "online": live["online"], "error": live["error"], "seen": live["seen"],
                            "cert_changed": live["cert_changed"], "action": live["action"],
                            "status": live["status"]})
            return out

    def history(self):
        with self.lock:
            return {dev_id: list(live["history"]) for dev_id, live in self.live.items() if dev_id in self.devices}

    # --- changes

    def add(self, address, user="", password="", token=""):
        address = address.strip()
        split_address(address)
        fingerprint = current_fingerprint(address)
        if not token:
            try:
                token = device_json(address, fingerprint, "POST", "/api/v1/token",
                                    body={"user": user, "password": password}).get("token", "")
            except DeviceError as e:
                if not is_old_agent(e):
                    raise
                web = WebLogin(address, fingerprint, user, password)
                try:
                    token = web.token()
                finally:
                    web.close()
        st = device_json(address, fingerprint, "GET", "/api/v1/status", token)
        info = st.get("device") or {}
        with self.lock:
            same = next((d for d in self.devices.values() if info.get("serial") and
                         d.get("serial") == info.get("serial")), None)
            dev = same or {"id": secrets.token_hex(6), "added": int(time.time())}
            dev.update(address=address, token=token, fingerprint=fingerprint, name=info.get("name", address),
                       serial=info.get("serial", ""))
            self.devices[dev["id"]] = dev
            self.save()
            self._record(dev["id"], st)
        log("device %s (%s) added" % (dev["name"], address))
        return dev["id"]

    def remove(self, ids):
        with self.lock:
            for dev_id in ids:
                if self.devices.pop(dev_id, None):
                    self.live.pop(dev_id, None)
            self.save()

    def trust(self, dev_id):
        with self.lock:
            live = self._live(dev_id)
            if dev_id not in self.devices or not live["cert_changed"]:
                raise DeviceError("nothing to trust")
            self.devices[dev_id]["fingerprint"] = live["cert_changed"]
            live.update(cert_changed=None, error="certificate trusted, waiting for the next poll")
            self.save()
        log("device %s: new certificate trusted" % dev_id)

    def set_action(self, dev_id, action, state, message=""):
        with self.lock:
            if dev_id in self.devices:
                self._live(dev_id)["action"] = {"action": action, "state": state, "message": message,
                                                "t": int(time.time())}

    def call(self, dev_id, method, path, body=None, **kw):
        dev = self.get(dev_id)
        return device_json(dev["address"], dev["fingerprint"], method, path, dev["token"], body, **kw)

    def api_version(self, dev_id):
        """From the last status seen: 1 for agents before 1.3.0 (no api_version), 0 if never seen."""
        with self.lock:
            st = self._live(dev_id)["status"]
            return st.get("api_version", 1) if st else 0


BULK = {"reboot": ("POST", "/api/v1/reboot", None), "restart": ("POST", "/api/v1/restart", None),
        "identify": ("POST", "/api/v1/identify", {"output": 0})}


def web_update(fleet, dev_id, package, downgrade, login):
    """Software update for an agent older than 1.3.0, through its web interface (needs a device login)."""
    if not login or not login[0]:
        raise DeviceError("the agent is older than 1.3.0: enter the device login next to the update")
    dev = fleet.get(dev_id)
    web = WebLogin(dev["address"], dev["fingerprint"], login[0], login[1])
    try:
        return {"message": web.update(package, downgrade)}
    finally:
        web.close()


def run_bulk(fleet, action, ids, package=None, downgrade=False, login=None):
    """In the background: each device's outcome shows in its "action"."""
    def one(dev_id):
        fleet.set_action(dev_id, action, "running", "uploading the package" if package else "")
        try:
            if package is not None and fleet.api_version(dev_id) >= 2:
                r = fleet.call(dev_id, "POST", "/api/v1/update%s" % ("?downgrade=1" if downgrade else ""),
                               package, ctype="application/octet-stream", timeout=UPDATE_TIMEOUT)
            elif package is not None:
                r = web_update(fleet, dev_id, package, downgrade, login)
            else:
                method, path, body = BULK[action]
                r = fleet.call(dev_id, method, path, body)
            fleet.set_action(dev_id, action, "ok", r.get("message", ""))
        except DeviceError as e:
            fleet.set_action(dev_id, action, "failed", str(e))

    def go():
        with concurrent.futures.ThreadPoolExecutor(4 if package else 16) as pool:
            list(pool.map(one, ids))
        log("%s on %d device(s) done" % (action, len(ids)))

    threading.Thread(target=go, daemon=True).start()


# ---------------------------------------------------------------- discovery

# MAC prefixes of Raspberry Pi boards (Raspberry Pi Foundation / Raspberry Pi Ltd). 98:FE:54 is on Pi 5 boards.
# A Pi on a USB Ethernet adapter has the adapter's prefix: such hosts show as "Linux with SSH".
RPI_OUIS = {"B8:27:EB", "DC:A6:32", "E4:5F:01", "D8:3A:DD", "28:CD:C1", "2C:CF:67", "88:A2:9E", "98:FE:54"}
SCAN_MAX_HOSTS = 1024


def local_network():
    """The /24 around the address this computer reaches other networks from (a guess the user can change)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("192.0.2.1", 9))  # no packet is sent: this only picks the outgoing interface
        return str(ipaddress.ip_network(s.getsockname()[0] + "/24", strict=False))
    except OSError:
        return "192.168.1.0/24"
    finally:
        s.close()


def arp_table():
    """IPv4 address -> MAC ("B8:27:EB:..") of the hosts this computer has talked to lately."""
    if os.path.exists("/proc/net/arp"):
        with open("/proc/net/arp") as f:
            text = f.read()
    else:
        try:
            text = subprocess.run(["arp", "-a"] if os.name == "nt" else ["arp", "-an"], capture_output=True,
                                  text=True, timeout=10).stdout
        except (OSError, subprocess.TimeoutExpired):
            return {}
    out = {}
    for m in re.finditer(r"(\d+\.\d+\.\d+\.\d+)\D+?((?:[0-9A-Fa-f]{1,2}[:-]){5}[0-9A-Fa-f]{1,2})\b", text):
        mac = ":".join(p.zfill(2) for p in re.split(r"[:-]", m.group(2))).upper()
        if mac not in ("00:00:00:00:00:00", "FF:FF:FF:FF:FF:FF"):
            out[m.group(1)] = mac
    return out


def probe(ip):
    """What answers on ip: the SSH banner, and whether HTTPS is PiNDI's web interface."""
    found = {}
    try:
        with socket.create_connection((ip, 22), 0.8) as s:
            s.settimeout(1.5)
            found["ssh"] = s.recv(256).split(b"\r\n")[0].decode("ascii", "replace")[:80]
    except OSError:
        pass
    try:
        with socket.create_connection((ip, 443), 0.8) as raw, NO_CA.wrap_socket(raw) as s:
            s.settimeout(2)
            s.sendall(b"HEAD /login HTTP/1.0\r\nHost: %s\r\n\r\n" % ip.encode())
            head = s.recv(1024).decode("latin-1")
            found["pindi"] = bool(re.search(r"^Server: PiNDI\b", head, re.M | re.I))
    except OSError:
        pass
    return found


class Scanner:
    """One scan at a time, in the background; the results stay until the next scan."""

    def __init__(self, fleet):
        self.fleet = fleet
        self.lock = threading.Lock()
        self.state = {"running": False, "network": local_network(), "results": [], "t": None, "error": ""}

    def start(self, network):
        try:
            net = ipaddress.ip_network(network.strip(), strict=False)
        except ValueError:
            raise DeviceError("not a network: %s (for example 192.168.1.0/24)" % network)
        if net.version != 4 or net.num_addresses > SCAN_MAX_HOSTS:
            raise DeviceError("scan an IPv4 network of at most %d addresses (/22)" % SCAN_MAX_HOSTS)
        with self.lock:
            if self.state["running"]:
                raise DeviceError("a scan is already running")
            self.state.update(running=True, network=str(net), error="")
        threading.Thread(target=self._run, args=(net,), daemon=True).start()

    def _run(self, net):
        hosts = [str(h) for h in (net.hosts() if net.num_addresses > 2 else net)]
        try:
            with concurrent.futures.ThreadPoolExecutor(128) as pool:
                probed = dict(zip(hosts, pool.map(probe, hosts)))
            arp = arp_table()  # filled by the probes above, also for hosts with nothing open
            managed = self.fleet.addresses()
            results = []
            for ip in hosts:
                p, mac = probed[ip], arp.get(ip, "")
                pi_mac = mac[:8] in RPI_OUIS
                if p.get("pindi"):
                    kind = "pindi"
                elif pi_mac:
                    kind = "pi"
                elif "ssh" in p:
                    kind = "ssh"
                else:
                    continue
                results.append({"ip": ip, "mac": mac, "pi_mac": pi_mac, "kind": kind, "ssh": p.get("ssh", ""),
                                "managed": managed.get(ip, "")})
            with self.lock:
                self.state.update(results=results, t=time.time())
            log("scan of %s: %d PiNDI, %d other Raspberry Pi" % (
                net, sum(r["kind"] == "pindi" for r in results), sum(r["kind"] == "pi" for r in results)))
        except Exception as e:  # shown in the GUI instead of a silent dead thread
            with self.lock:
                self.state["error"] = str(e)
        finally:
            with self.lock:
                self.state["running"] = False

    def snapshot(self):
        managed = self.fleet.addresses()
        with self.lock:
            out = dict(self.state)
            out["results"] = [dict(r, managed=managed.get(r["ip"], "")) for r in self.state["results"]]
            return out


# ---------------------------------------------------------------- installing PiNDI over SSH

# What install.sh needs (the same files make-update.sh packages), plus the NDI SDK the user downloaded.
AGENT_FILES = ["VERSION", "README.md", "install.sh", "uninstall.sh", "pindi.conf", "Splash.jpg", "update-key.pub",
               "pindi-player.c", "pindi-sender.c", "pindi-web.py", "pindi.service", "pindi-sender.service",
               "pindi-web.service", "pindi-webd.service"]
INSTALL_TIMEOUT = 3600  # install.sh with apt full-upgrade on a slow SD card
BOOT_WAIT = 300


def find_sdk(agent_dir):
    """The NDI SDK archive next to the agent (the Advanced SDK first, as install.sh prefers it)."""
    for kind in ("Advanced_SDK", "SDK"):
        hits = glob.glob(os.path.join(agent_dir, "Install_NDI_%s_v6_Linux.tar.gz" % kind))
        if hits:
            return hits[0]
    return ""


def ssh_available():
    try:
        import paramiko  # noqa: F401
        return True
    except ImportError:
        return False


class Installer:
    """Installs PiNDI on Raspberry Pis over SSH, one background job per address."""

    def __init__(self, fleet, agent_dir, known_hosts):
        self.fleet, self.agent_dir, self.known_hosts = fleet, agent_dir, known_hosts
        self.lock = threading.Lock()
        self.jobs = {}  # ip -> {"state", "step", "log", "t"}

    def status(self):
        with self.lock:
            return {ip: {k: j[k] for k in ("state", "step", "t")} for ip, j in self.jobs.items()}

    def log_of(self, ip):
        with self.lock:
            return "".join(self.jobs.get(ip, {}).get("log", []))[-50000:]

    def _say(self, ip, text, step=None):
        with self.lock:
            job = self.jobs[ip]
            job["log"].append(text if text.endswith("\n") else text + "\n")
            del job["log"][:-4000]
            if step:
                job["step"] = step

    def start(self, ips, user, password, skip_upgrade):
        if not ssh_available():
            raise DeviceError("installing needs the paramiko package: run  python -m pip install paramiko")
        sdk = find_sdk(self.agent_dir)
        if not sdk:
            raise DeviceError("put the NDI SDK (Install_NDI_SDK_v6_Linux.tar.gz from ndi.video/sdk) in %s"
                              % self.agent_dir)
        missing = [f for f in AGENT_FILES if not os.path.isfile(os.path.join(self.agent_dir, f))]
        if missing:
            raise DeviceError("the agent folder %s lacks %s" % (self.agent_dir, ", ".join(missing)))
        for ip in ips:
            ipaddress.IPv4Address(ip)
            with self.lock:
                if self.jobs.get(ip, {}).get("state") == "running":
                    continue
                self.jobs[ip] = {"state": "running", "step": "connecting", "log": [], "t": time.time()}
            threading.Thread(target=self._run, args=(ip, user, password, skip_upgrade, sdk), daemon=True).start()

    def _run(self, ip, user, password, skip_upgrade, sdk):
        try:
            self._install(ip, user, password, skip_upgrade, sdk)
            state = "ok"
        except Exception as e:  # paramiko, socket and our own errors: all end up in the job's log
            self._say(ip, "FAILED: %s" % (str(e) or e.__class__.__name__))
            state = "failed"
        with self.lock:
            self.jobs[ip].update(state=state, t=time.time())
        log("install on %s: %s" % (ip, state))

    def _install(self, ip, user, password, skip_upgrade, sdk):
        import paramiko

        installer = self

        class TrustFirstUse(paramiko.MissingHostKeyPolicy):
            """Unknown host: remember its key (a changed key is refused by paramiko itself)."""
            def missing_host_key(self, client, hostname, key):
                fp = base64.b64encode(hashlib.sha256(key.asbytes()).digest()).decode().rstrip("=")
                installer._say(ip, "New SSH host key %s SHA256:%s, remembered." % (key.get_name(), fp))
                client.get_host_keys().add(hostname, key.get_name(), key)
                with installer.lock:
                    client.save_host_keys(installer.known_hosts)

        client = paramiko.SSHClient()
        if os.path.exists(self.known_hosts):
            client.load_host_keys(self.known_hosts)
        client.set_missing_host_key_policy(TrustFirstUse())
        self._say(ip, "Connecting to %s@%s over SSH..." % (user, ip))
        try:
            client.connect(ip, username=user, password=password, timeout=10, allow_agent=False,
                           look_for_keys=False)
        except paramiko.BadHostKeyException:
            raise DeviceError("this address's SSH host key changed (reinstalled Pi?). If that's expected, delete "
                              "its line from %s and try again" % self.known_hosts)
        except paramiko.AuthenticationException:
            raise DeviceError("SSH login refused: check the user and password")
        try:
            def run(cmd):
                _, out, err = client.exec_command(cmd, timeout=30)
                return out.read().decode("utf-8", "replace").replace("\0", "").strip()

            model = run("cat /proc/device-tree/model 2>/dev/null")
            if "Raspberry Pi" not in model:
                raise DeviceError("not a Raspberry Pi (%s): nothing installed" % (model or "no device-tree model"))
            os_name = run(". /etc/os-release 2>/dev/null; echo \"$PRETTY_NAME\"")
            self._say(ip, "%s, %s" % (model, os_name), "uploading")

            sftp = client.open_sftp()
            try:
                try:
                    sftp.mkdir("pindi")
                except OSError:
                    pass  # already there (a reinstall)
                for name in AGENT_FILES + [os.path.basename(sdk)]:
                    self._say(ip, "Uploading %s" % name)
                    sftp.put(os.path.join(self.agent_dir, name), "pindi/" + name)
            finally:
                sftp.close()

            # sudo without a password (Raspberry Pi OS's first user) or with it on stdin.
            nopasswd = run("sudo -n true 2>/dev/null && echo yes") == "yes"
            cmd = "cd ~/pindi && sudo -S -p '' bash install.sh --accept-ndi-license --no-reboot%s 2>&1" % (
                " --skip-upgrade" if skip_upgrade else "")
            self._say(ip, "$ %s" % cmd, "installing")
            chan = client.get_transport().open_session()
            chan.settimeout(INSTALL_TIMEOUT)
            chan.exec_command(cmd)
            if not nopasswd:
                chan.sendall((password + "\n").encode())
            chan.shutdown_write()
            pending = b""
            while True:
                data = chan.recv(4096)
                if not data:
                    break
                pending += data
                *lines, pending = pending.split(b"\n")
                for line in lines:
                    self._say(ip, line.decode("utf-8", "replace").rstrip("\r"))
            if pending:
                self._say(ip, pending.decode("utf-8", "replace"))
            code = chan.recv_exit_status()
            if code:
                raise DeviceError("install.sh failed (exit code %d): see the log above" % code)

            self._say(ip, "Installed. Rebooting...", "rebooting")
            chan = client.get_transport().open_session()
            chan.exec_command("sudo -S -p '' systemctl reboot")
            if not nopasswd:
                chan.sendall((password + "\n").encode())
        finally:
            client.close()

        deadline = time.time() + BOOT_WAIT
        time.sleep(15)
        while not probe(ip).get("pindi"):
            if time.time() > deadline:
                raise DeviceError("installed, but the device hasn't come back after %d s: check its screen" % BOOT_WAIT)
            time.sleep(5)
        self._say(ip, "PiNDI is running. Adding it to the manager...", "adding")
        self.fleet.add(ip, user, password)
        self._say(ip, "Done.", "done")


# ---------------------------------------------------------------- the manager's own sign-in

class Auth:
    def __init__(self, path):
        self.path = path
        self.lock = threading.Lock()
        self.data = read_json(path, None)
        if not self.data:
            self.set_password(DEFAULT_USER, DEFAULT_PASSWORD)
        self.sessions = {}

    def set_password(self, user, password):
        salt = secrets.token_bytes(16)
        self.data = {"user": user, "salt": salt.hex(), "rounds": PBKDF2_ROUNDS,
                     "hash": hashlib.pbkdf2_hmac("sha256", password.encode(), salt, PBKDF2_ROUNDS).hex(),
                     "default": password == DEFAULT_PASSWORD}
        write_json(self.path, self.data)

    def check(self, user, password):
        d = self.data
        got = hashlib.pbkdf2_hmac("sha256", password.encode(), bytes.fromhex(d["salt"]), d["rounds"]).hex()
        return hmac.compare_digest(user.encode(), d["user"].encode()) & hmac.compare_digest(got, d["hash"])

    def login(self, user, password):
        if not self.check(user, password):
            time.sleep(1)
            return None
        token = secrets.token_urlsafe(32)
        with self.lock:
            now = time.time()
            self.sessions = {k: v for k, v in self.sessions.items() if v > now}
            self.sessions[token] = now + SESSION_HOURS * 3600
        return token

    def valid(self, token):
        with self.lock:
            return bool(token) and self.sessions.get(token, 0) > time.time()

    def logout(self, token):
        with self.lock:
            self.sessions.pop(token, None)

    def change(self, current, new, keep):
        if not self.check(self.data["user"], current):
            time.sleep(1)
            return "The current password is wrong."
        if len(new) < 4:
            return "The new password needs at least 4 characters."
        with self.lock:
            self.set_password(self.data["user"], new)
            self.sessions = {k: v for k, v in self.sessions.items() if k == keep}
        log("manager password changed")
        return ""


# ---------------------------------------------------------------- web server

class Handler(http.server.BaseHTTPRequestHandler):
    server_version = "PiNDI-Manager/" + VERSION
    fleet = auth = scanner = installer = None

    def log_message(self, fmt, *args):
        pass

    def send(self, code, body, ctype="application/json", headers=()):
        data = json.dumps(body).encode() if ctype == "application/json" else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        for k, v in headers:
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def session(self):
        c = http.cookies.SimpleCookie(self.headers.get("Cookie", ""))
        return c["pindi_manager"].value if "pindi_manager" in c else ""

    def body(self, limit=BODY_MAX):
        length = self.headers.get("Content-Length", "0").strip()
        if not length.isdigit() or int(length) > limit:
            self.send(413, {"error": "request too large" if length.isdigit() else "bad Content-Length"})
            return None
        return self.rfile.read(int(length))

    def json_body(self):
        raw = self.body()
        if raw is None:
            return None
        try:
            data = json.loads(raw or b"{}")
        except ValueError:
            data = None
        if not isinstance(data, dict):
            self.send(400, {"error": "the body must be a JSON object"})
            return None
        return data

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        if not url.path.startswith("/api/"):
            return self.static(url.path)
        if not self.auth.valid(self.session()):
            return self.send(401, {"error": "sign in"})
        q = {k: v[0] for k, v in urllib.parse.parse_qs(url.query).items()}
        if url.path == "/api/session":
            return self.send(200, {"user": self.auth.data["user"], "default_password": self.auth.data["default"],
                                   "version": VERSION, "poll_sec": POLL_SEC})
        if url.path == "/api/state":
            return self.send(200, {"devices": self.fleet.state(), "t": time.time()})
        if url.path == "/api/history":
            return self.send(200, self.fleet.history())
        if url.path == "/api/thumb":
            try:
                dev = self.fleet.get(q.get("id", ""))
                n = int(q.get("n", "1"))
                status, data, ctype = device_request(dev["address"], dev["fingerprint"], "GET",
                                                     "/api/v1/outputs/%d/thumb.jpg" % n, dev["token"])
            except (DeviceError, ValueError):
                return self.send(404, b"", "image/jpeg")
            return self.send(status if status == 200 else 404, data if status == 200 else b"", "image/jpeg")
        if url.path == "/api/update-log":
            try:
                return self.send(200, self.fleet.call(q.get("id", ""), "GET", "/api/v1/update"))
            except DeviceError as e:
                return self.send(502, {"error": str(e)})
        if url.path == "/api/discover":
            sdk = find_sdk(self.installer.agent_dir)
            return self.send(200, {"scan": self.scanner.snapshot(), "installs": self.installer.status(),
                                   "sdk": os.path.basename(sdk), "ssh": ssh_available(),
                                   "agent_dir": self.installer.agent_dir})
        if url.path == "/api/install-log":
            return self.send(200, {"log": self.installer.log_of(q.get("ip", ""))})
        self.send(404, {"error": "unknown"})

    def do_POST(self):
        url = urllib.parse.urlparse(self.path)
        # Cross-site forms can't set custom headers: together with the SameSite cookie, no CSRF.
        if self.headers.get("X-PiNDI-Manager") != "1":
            return self.send(403, {"error": "missing X-PiNDI-Manager header"})
        if url.path == "/api/login":
            data = self.json_body()
            if data is None:
                return
            token = self.auth.login(str(data.get("user", "")), str(data.get("password", "")))
            if not token:
                log("manager sign-in failed from %s" % self.client_address[0])
                return self.send(401, {"error": "Wrong user or password."})
            secure = "; Secure" if isinstance(self.connection, ssl.SSLSocket) else ""
            return self.send(200, {"ok": True}, headers=[(
                "Set-Cookie", "pindi_manager=%s; Path=/; HttpOnly; SameSite=Strict%s" % (token, secure))])
        token = self.session()
        if not self.auth.valid(token):
            return self.send(401, {"error": "sign in"})
        if url.path == "/api/update":
            return self.update(url)
        data = self.json_body()
        if data is None:
            return
        ids = [str(i) for i in data.get("ids", []) if isinstance(i, str)]
        try:
            if url.path == "/api/logout":
                self.auth.logout(token)
                return self.send(200, {"ok": True}, headers=[("Set-Cookie", "pindi_manager=; Path=/; Max-Age=0")])
            if url.path == "/api/password":
                err = self.auth.change(str(data.get("current", "")), str(data.get("new", "")), token)
                return self.send(400 if err else 200, {"error": err} if err else {"ok": True})
            if url.path == "/api/devices":
                dev_id = self.fleet.add(str(data.get("address", "")), str(data.get("user", "")),
                                        str(data.get("password", "")), str(data.get("token", "")).strip())
                return self.send(200, {"ok": True, "id": dev_id})
            if url.path == "/api/devices/remove":
                self.fleet.remove(ids)
                return self.send(200, {"ok": True})
            if url.path == "/api/devices/trust":
                self.fleet.trust(str(data.get("id", "")))
                return self.send(200, {"ok": True})
            if url.path == "/api/route":
                r = self.fleet.call(str(data.get("id", "")), "POST", "/api/v1/outputs/%d/source" % int(data["output"]),
                                    {"stream": str(data.get("stream", "")), "source_ip": str(data.get("source_ip", ""))})
                self.fleet.poll(self.fleet.get(str(data.get("id", ""))))  # show it at once
                return self.send(200, r)
            if url.path == "/api/bulk":
                action = data.get("action")
                if action not in BULK or not ids:
                    return self.send(400, {"error": "choose devices and an action"})
                run_bulk(self.fleet, action, ids)
                return self.send(202, {"ok": True})
            if url.path == "/api/discover":
                self.scanner.start(str(data.get("network", "")))
                return self.send(202, {"ok": True})
            if url.path == "/api/install":
                if data.get("accept_ndi_license") is not True:
                    return self.send(400, {"error": "accept the NDI SDK license first"})
                ips = [str(i) for i in data.get("ips", []) if isinstance(i, str)]
                if not ips:
                    return self.send(400, {"error": "choose the Raspberry Pis to install on"})
                self.installer.start(ips, str(data.get("user", "")), str(data.get("password", "")),
                                     bool(data.get("skip_upgrade")))
                return self.send(202, {"ok": True})
        except (DeviceError, KeyError, ValueError, TypeError) as e:
            return self.send(400, {"error": str(e) or "bad request"})
        self.send(404, {"error": "unknown"})

    def update(self, url):
        """The update package is the body; the devices are in the query (?ids=a,b&downgrade=1)."""
        q = {k: v[0] for k, v in urllib.parse.parse_qs(url.query).items()}
        ids = [i for i in q.get("ids", "").split(",") if i]
        package = self.body(UPDATE_MAX)
        if package is None:
            return
        if not ids or not package.startswith(b"\x1f\x8b"):
            return self.send(400, {"error": "choose devices and a PiNDI update package (.tar.gz)"})
        # The device login is only needed for agents older than 1.3.0 (no update API): headers, not the URL.
        login = (self.headers.get("X-Device-User", ""), self.headers.get("X-Device-Password", ""))
        run_bulk(self.fleet, "update", ids, package, q.get("downgrade") == "1", login)
        self.send(202, {"ok": True})

    def static(self, path):
        name = "index.html" if path in ("/", "") else path.lstrip("/")
        full = os.path.realpath(os.path.join(STATIC_DIR, name))
        if not full.startswith(os.path.realpath(STATIC_DIR) + os.sep) or not os.path.isfile(full):
            return self.send(404, b"Not found", "text/plain")
        with open(full, "rb") as f:
            data = f.read()
        ctype = mimetypes.guess_type(full)[0] or "application/octet-stream"
        self.send(200, data, ctype + ("; charset=utf-8" if ctype.startswith("text/") or ctype.endswith("javascript")
                                      else ""),
                  [("Content-Security-Policy", "default-src 'self'; img-src 'self' data:; style-src 'self' 'unsafe-inline'")])


class Server(http.server.ThreadingHTTPServer):
    daemon_threads = True
    # On Windows SO_REUSEADDR lets a second process bind the same port and take over its connections.
    allow_reuse_address = os.name != "nt"


def main():
    ap = argparse.ArgumentParser(description="PiNDI Endpoint Manager")
    ap.add_argument("--host", default="127.0.0.1", help="address to listen on (default 127.0.0.1: this computer only)")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--data", default=default_data_dir(),
                    help="folder for devices.json, auth.json and known_hosts (default: %(default)s)")
    ap.add_argument("--agent", default=os.path.join(HERE, "..", "agent"),
                    help="the agent folder installed on new Raspberry Pis, with the NDI SDK in it (default ../agent)")
    ap.add_argument("--cert", help="TLS certificate (PEM) to serve HTTPS")
    ap.add_argument("--key", help="its private key (PEM)")
    args = ap.parse_args()
    if bool(args.cert) != bool(args.key):
        ap.error("--cert and --key go together")
    if args.host not in ("127.0.0.1", "localhost", "::1") and not args.cert:
        print("Warning: listening on %s without --cert: passwords cross the network unencrypted." % args.host,
              file=sys.stderr)
    os.makedirs(args.data, mode=0o700, exist_ok=True)
    lock_down(args.data)
    Handler.fleet = Fleet(os.path.join(args.data, "devices.json"))
    Handler.auth = Auth(os.path.join(args.data, "auth.json"))
    Handler.scanner = Scanner(Handler.fleet)
    Handler.installer = Installer(Handler.fleet, os.path.realpath(args.agent), os.path.join(args.data, "known_hosts"))
    log("data in %s (this user only)" % args.data)
    threading.Thread(target=Handler.fleet.poll_loop, daemon=True).start()
    try:
        srv = Server((args.host, args.port), Handler)
    except OSError as e:
        sys.exit("cannot listen on %s:%d (%s): is the manager already running?" % (args.host, args.port, e))
    scheme = "http"
    if args.cert:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(args.cert, args.key)
        srv.socket = ctx.wrap_socket(srv.socket, server_side=True)
        scheme = "https"
    log("PiNDI Endpoint Manager %s on %s://%s:%d/" % (VERSION, scheme, args.host, args.port))
    if Handler.auth.data["default"]:
        log("sign in with %s / %s and change the password (Admin -> Manager)" % (DEFAULT_USER, DEFAULT_PASSWORD))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
