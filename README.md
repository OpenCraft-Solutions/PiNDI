# PiNDI

Raspberry Pi NDI display decoders and encoders, and the tools to run a fleet of them.

| Folder | What it is |
| --- | --- |
| [agent/](agent/) | The software installed on each Raspberry Pi: NDI decoder/encoder, splash screen, web interface and HTTP API. See [agent/README.md](agent/README.md). |
| [endpoint-manager/](endpoint-manager/) | Web GUI for many devices: monitoring, a video matrix and bulk admin (reboot, update, ...). See [endpoint-manager/README.md](endpoint-manager/README.md). |
| [docs/](docs/) | Documentation shared between components, including the [agent's HTTP API](docs/agent-api.md). |

Each component has its own `VERSION` file and is released on its own, tagged
`<component>-v<version>` (for example `agent-v1.2.0`).
