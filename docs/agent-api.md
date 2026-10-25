# PiNDI agent HTTP API

The API every PiNDI device serves, and that the endpoint manager is built against. Implemented in
`agent/pindi-web.py` (`api_status`, `api_post` and the `/api/v1/` routes in `Handler`).

## Basics

- **HTTPS only**, port 443. A request carrying the valid token over plain HTTP gets a 403 and the token is
  replaced at once, because it crossed the network unencrypted.
- **Certificates** are self-signed unless one was uploaded. Pin the device's certificate or public key; don't
  just switch checks off. The manager pins the certificate's SHA-256 fingerprint. A device makes a new
  self-signed certificate only when it gets a host name or address its current one doesn't cover (not at
  every boot), and keeps its key, so a client that pins the public key keeps trusting it.
- **Authentication:** `Authorization: Bearer <token>` or `X-API-Key: <token>`. Tokens in the URL aren't
  accepted. Wrong tokens are answered after a second and never lock anyone out.
- **Bodies** are JSON or form data (except the update package, below). Answers are JSON.
- **Versioning:** everything is under `/api/v1/` and nothing is removed. `api_version` in the status
  rises when something is added. It is missing (treat it as 1) before agent 1.3.0.

| `api_version` | Agent | Added |
| --- | --- | --- |
| 1 | ≤ 1.2 | status, output source, identify, restart |
| 2 | 1.3.0 | `api_version`, `device.metrics`, `device.update`; reboot, update, update log, thumbnails, token from a sign-in |
| 3 | 1.4.0 | shutdown |

## Requests

### `POST /api/v1/token` (no token: a sign-in)

```json
{"user": "pindi", "password": "pindi"}
```

→ `200 {"token": "..."}` for a member of the device's `sudo` group. `401` for a wrong login, `429` while that user
or address is locked out (the same lockouts as the sign-in page: 5 failures per user and address, 30 per address,
in 10 minutes). The token is the device's single API token, the same one shown in Device settings.

### `GET /api/v1/status`

```jsonc
{
 "api_version": 3,
 "device": {
  "name": "Lobby left", "ndi_name": "LOBBY-LEFT", "serial": "10000000abcd1234", "model": "Raspberry Pi 4 Model B",
  "version": "1.3.0", "ip": "192.168.1.50",
  "health": "CPU 52 °C · load 0.8 (4 cores) · memory 31% used · storage 40% used · up 3 d 2 h",
  "power": "OK",                       // or the under-voltage / throttling warning text
  "network": "41.2 Mbit/s in, 0.3 Mbit/s out (1000 Mbit/s link)",
  "metrics": {                         // numbers for graphs; null where unknown
   "cpu_temp": 52.1, "load": 0.81, "cores": 4, "memory_pct": 31.0, "storage_pct": 40.2, "uptime": 266400,
   "throttled": 0,                     // vcgencmd get_throttled bits
   "rx_bps": 41200000, "tx_bps": 300000, "link_mbps": 1000
  },
  "update": ""                         // software update: "running", "failed", "done" (rebooting) or ""
 },
 "outputs": [{
  "n": 1, "connector": "HDMI-A-1", "enabled": true,
  "state": "playing",                  // or "waiting for <stream>", "player not running", ...
  "target": "CAM 2",                   // the stream it shows (OUT_<serial>[_n] by default)
  "stream": "CAM 2",                   // the setting: "" = the default
  "source_ip": "192.168.1.21",         // "" = any sender
  "backup": "", "source": "STUDIO-A (CAM 2)", "video": "1920x1080 59.94p", "format": "Full NDI (SpeedHQ)",
  "audio": "48 kHz stereo", "latency": "2 frames",
  "display": {"link": "connected", "mode": "1920x1080@60"}
 }],
 "inputs": [{
  "slot": 1, "device": "/dev/v4l/by-id/...", "present": true, "enabled": true, "name": "IN_10000000abcd1234",
  "state": "ON AIR", "mode": "MJPG 1920x1080@30", "fps": "30.0", "audio": "48 kHz stereo", "on_air": true
 }],
 "sources": [                          // every NDI source this device sees
  {"name": "STUDIO-A (CAM 2)", "stream": "CAM 2", "ip": "192.168.1.21"}
 ]
}
```

Cheap enough to poll every few seconds. It only reads status files and doesn't enumerate devices.

### `POST /api/v1/outputs/<n>/source`

```json
{"stream": "CAM 2", "source_ip": "192.168.1.21", "backup": ""}
```

Any subset. The output switches live and the setting is saved. A stream is matched by the part of the NDI
name in brackets, case-insensitively. With `source_ip`, only from that sender. `"stream": ""` is the default
stream, `OUT_<serial>[_n]`. → `200 {"ok": true, "output": 1, "stream": "CAM 2", "source_ip": "...", "backup": ""}`,
`404` for an unknown output, `400` for an invalid name or address.

### `GET /api/v1/outputs/<n>/thumb.jpg`

Output *n*'s current picture as JPEG. Asking for it keeps the player rendering thumbnails for 15 s, so the first
request after a while may get a `404`.

### `POST /api/v1/identify`

`{"output": 1}`, or 0 for all outputs. The device name, IP and serial show on the screen for 15 s.

### `POST /api/v1/restart`

Restarts the video and outgoing streams (`pindi` and `pindi-sender` services).

### `POST /api/v1/reboot`

Reboots the device about a second after answering `200 {"ok": true}`.

### `POST /api/v1/shutdown`

Shuts the device down about a second after answering `200 {"ok": true}` (API version 3). Nothing on the
network can start it again: it stays off until its power is unplugged and plugged back in.

### `POST /api/v1/update`

The body is a signed update package from `agent/make-update.sh`, as `application/octet-stream`, at most 32 MB.
`?downgrade=1` allows an older version than the installed one. The device checks the signature against
the `update-key.pub` it was installed with, then installs in the background and reboots.
→ `200 {"ok": true, "message": "Installing PiNDI 1.3.1. ..."}`, `400 {"ok": false, "message": "Update refused: ..."}`,
`409` while an update is already running.

### `GET /api/v1/update`

`{"state": "running" | "failed" | "done" | "", "log": "<last 20 000 characters>"}`. The state is `""` when no
update has run since the last boot.
