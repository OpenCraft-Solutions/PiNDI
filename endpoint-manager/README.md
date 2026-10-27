# PiNDI Endpoint Manager

A web GUI for many PiNDI devices ([agent/](../agent/)) at once:

- **Monitoring:** devices online, outputs showing a stream, streams on air and alerts; graphs of CPU
  temperature, load, network traffic and memory for the last hour; tables of every device, display output
  and outgoing stream.
- **Video matrix:** every NDI source your PiNDI devices can see, and their H.264 RTP multicast and SRT streams (rows),
  against every PiNDI display output (columns). Click a crosspoint to switch that output live. Optional previews
  show what each output shows.
- **Admin:** find Raspberry Pis on the network and install PiNDI on them; select devices, then identify,
  restart video, reboot, shut down (agent 1.4.0 or newer; a Pi that is shut down stays off until its power is
  cycled), install a software update or remove them; trust a device's new certificate; change
  the manager's password.

Python 3.8 or newer. Everything uses the standard library except installing PiNDI on new Pis over SSH, which
needs `paramiko` (`python -m pip install -r requirements.txt`). It runs on Windows, macOS, Linux or a spare
Raspberry Pi.

## Run

```sh
python3 manager.py
```

Open http://127.0.0.1:8080/ and sign in with **pindi / pindi**. Change the password under
**Admin → Manager** (a banner reminds you until you do).

By default the manager only listens on this computer. To use it from other computers, give it a
certificate so passwords don't cross the network unencrypted:

```sh
python3 manager.py --host 0.0.0.0 --port 8443 --cert cert.pem --key key.pem
```

| Option | Default | |
| --- | --- | --- |
| `--host` | `127.0.0.1` | Address to listen on |
| `--port` | `8080` | |
| `--data` | `%LOCALAPPDATA%\PiNDI Manager` (Windows), `~/.config/pindi-manager` | Saved data, see [What is saved](#what-is-saved) |
| `--agent` | `../agent` | The agent installed on new Pis; the NDI SDK goes in it |
| `--cert`, `--key` | | Serve HTTPS with this certificate and key (PEM) |

## Finding and adding devices

**Admin → Find Raspberry Pis** scans a network (the one this computer is on, or any `/22` or smaller) in a few
seconds and lists:

- **PiNDI**: devices running PiNDI (their web interface answers as PiNDI). Tick them and press **Add selected
  PiNDI**.
- **Raspberry Pi**: Pis without PiNDI, recognised by the MAC address of their network port. Tick them and
  press **Install PiNDI on selected**.
- **Linux with SSH** (shown with *Other SSH hosts*): other machines with SSH, including Pis on a USB Ethernet
  adapter, whose MAC address doesn't give them away. Install checks over SSH that a host really is a
  Raspberry Pi before changing anything.

Adding a device takes an administrator login on the device (`pindi` / `pindi` unless you changed it). The
manager uses it once, to fetch the device's API token, and doesn't keep it. That works on every agent version:
1.3.0 and newer hand out the token through the API, older ones through their web interface. You can also add
a device by address under **Add a device**, or paste its API token (Device settings → HTTP API) instead of a
login.

## Installing PiNDI on a new Raspberry Pi

1. Write Raspberry Pi OS Lite to the SD card with Raspberry Pi Imager, with a user and password and **SSH
   enabled** (as in the agent's README).
2. Download the NDI SDK for Linux from https://ndi.video/sdk/ (it can't be included) and put
   `Install_NDI_SDK_v6_Linux.tar.gz` in the `agent/` folder (or the Advanced SDK archive for NDI|HX receive).
3. Scan, tick the Pis, enter their login, accept the NDI SDK license and press **Install PiNDI on selected**.

For each Pi the manager logs in over SSH, checks it's a Raspberry Pi, uploads the agent and the SDK to
`~/pindi`, runs `install.sh` (with `apt full-upgrade` unless you tick *Skip the OS upgrade*), reboots it and
adds it to the manager once PiNDI answers. The **log** link follows each step. Expect 5 to 30 minutes,
mostly the OS upgrade.

## Software updates

Build a signed package with `agent/make-update.sh`, choose it under **Admin → Software update**, select the
devices and press **Install on selected**. Each device checks the signature, installs and reboots (about 2
minutes). Agents 1.3.0 and newer take the package through the API, and the **Software update** column and its
**log** link show the progress. Older agents take it through their web interface, using the device login next
to the button, so you can bring them all to 1.3.0 from here.

## What is saved

Everything the manager saves is in its data folder, outside the repository so it is never committed:

| File | Holds | Protected by |
| --- | --- | --- |
| `devices.json` | Device addresses, names, certificate fingerprints, API tokens | Tokens encrypted with your Windows account's key (DPAPI): unreadable to other accounts and on other computers |
| `auth.json` | The manager's password, as a salted PBKDF2-SHA256 hash (300 000 rounds) | Can't be turned back into the password |
| `known_hosts` | SSH host keys of the Pis PiNDI was installed on | Not secret; it detects an impostor |

The folder is restricted to your user account (on Windows also SYSTEM and Administrators, as for Windows
OpenSSH's keys; on Linux and macOS mode `700`). On Linux and macOS there is no DPAPI, so the folder's
permissions are what protect the tokens there.

Never saved: device passwords (used for one add, install or update, then forgotten), the manager's password
in plain text, and scan results (kept in memory only). Sessions are in memory and end after 12 hours or when
the manager stops.

### Certificates and host keys

Devices make their own (self-signed) certificates. When a device is added, the manager remembers the SHA-256
fingerprint of the certificate it presents, and checks it before every request, so no API token is ever
sent to an impostor. A device makes a new certificate when its name or address changes. The manager then
shows **certificate changed** and stops talking to it until you press **Trust new certificate** in Admin.
Compare the fingerprint with **Device settings → HTTPS certificate** on the device first.

SSH host keys work the same way: remembered on the first install, and a changed key is refused (delete the
Pi's line from `known_hosts` after reinstalling its SD card).

## How the matrix works

The manager has no NDI library of its own. The source list is every source that any managed device sees
(each device's `/api/v1/status` lists them), so it covers NDI Discovery Servers and other subnets wherever
your devices see them. Sources sent by managed devices are tagged *PiNDI*.

Inputs that managed devices send as **H.264 RTP to a multicast group** are listed too (tagged *H.264 RTP*):
routing one sets the output's stream to `rtp://<group>:<port>`, and the output joins the group. RTP sent to a
single address (unicast) goes to one receiver chosen on the sending device, so it isn't offered here.

Inputs sent as **H.264 SRT** that listen for receivers (`srt://:<port>`) are listed as well (tagged *H.264 SRT*):
routing one sets the output's stream to `srt://<device>:<port>`, and the output calls the sending device (up to
8 outputs per input). Encrypted ones aren't offered, because devices don't give their passphrase out, and
neither are inputs that call one receiver themselves: set those on the device.

RTP and SRT crosspoints are disabled for outputs on devices older than PiNDI 1.7.0, which can't show them.

Switching sends `POST /api/v1/outputs/<n>/source` to the device. With **Lock to the sender's IP** (the default),
the output only accepts that stream from that sender, so two senders with the same stream name can't be
mixed up. **Own stream** sets the output back to its default stream, `OUT_<serial>`.

## Development

`dev/mock_agent.py` runs fake devices that speak the agent API, so you can work on the manager without
Raspberry Pis (it needs the `openssl` command for their certificates):

```sh
python3 dev/mock_agent.py --count 3     # devices at 127.0.0.1:9441-9443, login pindi / pindi
python3 manager.py --data /tmp/pindi-manager-dev
```

The agent API the manager relies on is described in [docs/agent-api.md](../docs/agent-api.md).
