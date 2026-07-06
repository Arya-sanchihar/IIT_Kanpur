# BATMAN-Adv Mesh Video Network

**Comms subsystem** — Infrastructure-independent P2P streaming across Raspberry Pi nodes.

## Problem Statement

Centralized comms (cellular, fixed Wi-Fi APs) create a single point of failure in a contested environment. A soldier-worn camera and multi-Pi mesh must route peer-to-peer video and telemetry with no central control, gracefully handling nodes joining, leaving, or moving out of range.

## Methodology

- **Layer-2 mesh**: B.A.T.M.A.N.-Adv over ad-hoc `wlan0`, joined into a `bat0` virtual-switch interface
- **Zero-config forwarding**: Avahi mDNS link-local IPs — no DHCP server required
- **Video path**: GStreamer producer/consumer, H.264 over RTP/UDP; GTK receiver; suppressed monitoring UI
- Validated first in a 3-container Docker simulation, then on 3 physical Raspberry Pi nodes (Pi 4 + Pi 5) with on-node OSD displays

## Results

Logical routing paths through mesh nodes & interfaces, illustrating best-next-hop selection independent of centralized routing. See `docs/` (top-level) for the mesh topology diagram.

## Novelty

Operates at the data-link layer, not IP — the whole mesh presents as one virtual switch, so every node need only find its best next hop rather than compute a full path. Combined with DHCP-free Avahi addressing, this gives fully self-organizing, infrastructure-independent bring-up, with each node doubling as both a relay and a local display endpoint.

## GTK Monitoring UI

A custom GTK4 desktop client drives the GStreamer consumer pipeline (`rtph264depay → avdec_h264 → gtksink`) for live per-node video preview. Each Pi runs the matching producer pipeline (`v4l2src → x264enc → rtph264pay → udpsink`) that feeds the mesh.

## Repository Layout (suggested)

```
01-batman-adv-mesh-network/
├── README.md
├── docker/              # 3-container simulation setup
├── pi-config/           # bat0 / batman-adv / Avahi configuration scripts
├── gstreamer/           # producer & consumer pipeline scripts
├── gtk-ui/              # GTK4 monitoring client source
└── docs/                # topology diagrams, test logs
```

> Populate the folders above with your actual scripts, Dockerfiles, and configs, then commit.
