# aes67

AES67 / RAVENNA audio-over-IP for the **ESP32-P4**.

An audio-over-IP endpoint — PTP-synchronized RTP audio with SAP/SDP
discovery — running on a microcontroller-class SoC. The clock is
disciplined off the ESP32-P4 EMAC's IEEE-1588 hardware timestamping, audio
is moved by ISR-driven I2S DMA, and the receive path can bypass lwIP for
low latency.

Interop-tested on real hardware against **Merging Technologies SIENNA**
(commercial RAVENNA) and the **aes67-linux-daemon** running on a Raspberry
Pi.

> **ESP32-P4 only.** The implementation depends on the P4 EMAC's hardware
> PTP timestamping and the P4 I2S/APLL clocking. It is not portable to
> other ESP32 targets as-is.

## What this component provides

The component is the AES67 **engine** plus the drivers it needs:

- **PTP (IEEE-1588v2)** clock sync off the EMAC hardware timestamp unit —
  slave mode (locks to an external grandmaster) and grandmaster mode (BMCA
  promotes this node to GM when no better clock is present). The PTP clock
  identity is derived from the MAC via EUI-64.
- **RTP engine** — source (TX) and sink (RX), L16 / L24 / L32 / AM824
  codecs via a function-pointer conversion dispatch, 1–8 channels, 44.1 and
  48 kHz, packet times from 0.125 ms to 4 ms.
- **An `esp_vfs_l2tap` receive path** that reads RTP frames at L2 and
  bypasses the lwIP socket layer for low-latency reception.
- **ISR-driven I2S playback** on an APLL-derived clock, with sub-ppm clock
  adjustment for media-clock recovery and sample-hold on underrun.
- **SAP (RFC 2974)** announce/discover, **SDP** generate/parse
  (AES67-compliant: `a=ts-refclk`, `a=mediaclk`, fractional ptime),
  **mDNS** `_ravenna._udp` advertisement.
- An optional **embedded Web UI** (gzipped SPA + WebSocket) for live status
  and stream management.

### What lives in the example, not the component

The repository's [`main/`](https://github.com/DatanoiseTV/aes67-esp32p4/tree/main/main)
example shows the board bring-up (Ethernet/PHY, ES8311 codec) and one extra
integration: an **`esp_eth` Rx hook** that intercepts RTP frames in the
driver callback — before lwIP — for the absolute lowest latency (the ~0.7 ms
figure). That hook is example code you copy and adapt to your board, not a
component API. The component's own `esp_vfs_l2tap` path is the portable
low-latency option.

## Quick start

```bash
idf.py add-dependency "datanoisetv/aes67^0.1.0"
```

```c
#include "aes67.h"
#include "aes67_session.h"

aes67_config_t cfg = AES67_CONFIG_DEFAULT();
cfg.net.eth_handle = my_eth_handle;     /* you install esp_eth yourself */

aes67_node_handle_t node;
aes67_node_init(&cfg, &node);
aes67_node_start(node);

aes67_session_handle_t session;
aes67_node_get_session(node, &session);

uint8_t id;
aes67_session_add_source(session, "My Source", 2, AES67_CODEC_L24, &id);
/* add a sink from a discovered SDP: */
aes67_session_add_sink(session, "My Sink", remote_sdp_text, &id);
```

A complete, board-specific example (ESP32-P4-Nano + ES8311) lives in the
[repository](https://github.com/DatanoiseTV/aes67-esp32p4).

## Configuration

Options live under **AES67 RAVENNA Component** and **PTP Daemon
Configuration** in `idf.py menuconfig` — max sources/sinks, channels,
sample rate, packet time, codec, RTP port, PTP domain, jitter-buffer depth,
PSRAM usage, and task priorities.

For low-latency audio the example's `sdkconfig.defaults` also enables
802.3x flow control (`CONFIG_ETH_*`), lwIP IRAM optimizations, and pins the
network tasks to core 0 — see the repository for the full set.

## Requirements & limitations

- **ESP-IDF v5.5+**, target **esp32p4**.
- Sample rates **44.1 / 48 kHz** (no 96 kHz yet).
- The lowest-latency `esp_eth` Rx hook is example wiring, not a component
  API (see above).
- The bundled Web UI is optional and can be left out.

## Bundled third-party code

To keep the component self-contained on the registry, it vendors two
Apache-2.0 components, with their license headers and per-file provenance
intact:

- `third_party/ptpd/` — a minimal IEEE-1588 PTP daemon (Apache NuttX
  `ptpd`, ported to ESP-IDF by Espressif).
- `third_party/esp_eth_time/` — the EMAC time-control wrapper from the
  ESP-IDF `ethernet/ptp` example (Espressif).

See [`NOTICE`](NOTICE) and the SPDX headers in those files.

## License

Apache-2.0. See the SPDX headers and `NOTICE`.
