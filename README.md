# AES67/RAVENNA for ESP32-P4

An ESP-IDF component implementing the AES67 (and RAVENNA-compatible) audio-over-IP standard on the ESP32-P4, ported from the [aes67-linux-daemon](https://github.com/bondagit/aes67-linux-daemon) reference implementation.

## Features

- **PTP (IEEE 1588v2)** clock synchronization using ESP32-P4 EMAC hardware timestamping
  - Slave mode: synchronizes to an external PTP grandmaster
  - Grandmaster mode: becomes PTP GM when no other master is present on the network
  - Sub-microsecond lock detection with configurable thresholds
- **RTP audio streaming** with per-stream jitter buffers
  - Source (TX) and Sink (RX) streams
  - L16, L24, L32, AM824 codec support
  - Configurable packet time (125us to 4ms, default 1ms per AES67)
  - DSCP/QoS marking for network prioritization
- **SAP (RFC 2974)** session announcement and discovery
  - Periodic multicast announcements for source streams
  - Remote source discovery with timeout-based expiration
- **SDP** generation and parsing (AES67-compliant)
  - PTP reference clock attributes (`a=ts-refclk`)
  - Media clock attributes (`a=mediaclk`)
  - Fractional ptime support for sub-millisecond packet times
- **mDNS/DNS-SD** service advertisement (`_ravenna._udp`)
- **I2S audio** driver with DMA
  - Standard I2S Philips mode
  - Ring buffers in PSRAM for large buffer capacity
  - DMA buffers in internal SRAM for reliable transfers
- **PSRAM support** for audio ring buffers and jitter buffers

## Hardware

The example targets the **ESP32-P4-NANO** board with:

| Function | GPIO |
|---|---|
| Ethernet MDC | 31 |
| Ethernet MDIO | 52 |
| Ethernet REF_CLK | 50 |
| Ethernet PHY_RST | 51 |
| I2S MCLK | 13 |
| I2S SCLK | 12 |
| I2S LRCK | 10 |
| I2S DOUT (ESP->codec) | 11 |
| I2S DIN (codec->ESP) | 9 |
| PA Enable | 53 |

Ethernet PHY: IP101 GRI via RMII, external 50MHz crystal.
Audio codec: ES8311 via I2S.

## Build

Requires ESP-IDF v5.5+ with the ESP32-P4 target.

```bash
idf.py set-target esp32p4
idf.py build
idf.py flash monitor
```

## Component Architecture

```
components/aes67/
  include/
    aes67.h             Top-level init/lifecycle API
    aes67_config.h      Configuration structs and defaults
    aes67_ptp.h         PTP clock synchronization
    aes67_rtp.h         RTP packet engine
    aes67_sap.h         SAP announcement/discovery
    aes67_sdp.h         SDP parser/generator
    aes67_session.h     Session manager (sources/sinks)
    aes67_audio.h       I2S audio driver
    aes67_convert.h     Sample format conversion
    aes67_net.h         Network utilities
  src/
    aes67.c             Node lifecycle management
    aes67_ptp.c         PTP wrapping ptpd + esp_eth_time
    aes67_rtp.c         RTP engine with ring buffers
    aes67_sap.c         SAP multicast announcements
    aes67_sdp.c         SDP text generation/parsing
    aes67_session.c     Stream orchestration
    aes67_audio.c       I2S DMA driver
    aes67_convert.c     L16/L24/L32 format conversion
    aes67_net.c         Multicast socket helpers
    aes67_mdns.c        mDNS service registration
```

## Configuration

All parameters are configurable via `idf.py menuconfig` under "AES67 RAVENNA Component":

- Max sources/sinks (default: 4 each)
- Max channels per stream (default: 8)
- Sample rate (default: 48000 Hz)
- Packet time (default: 1000us)
- Word length (default: 3 = L24)
- RTP port (default: 5004)
- PTP domain (default: 0)
- Jitter buffer depth (default: 4ms)
- PSRAM usage for buffers
- I2S pin assignments
- Task priorities

## Usage

```c
#include "aes67.h"
#include "aes67_session.h"

/* Configure and start */
aes67_config_t cfg = AES67_CONFIG_DEFAULT();
cfg.net.eth_handle = my_eth_handle;

aes67_node_handle_t node;
aes67_node_init(&cfg, &node);
aes67_node_start(node);

/* Add a 2-channel L24 source */
aes67_session_handle_t session;
aes67_node_get_session(node, &session);

uint8_t id;
aes67_session_add_source(session, "My Source", 2, AES67_CODEC_L24, &id);

/* Add a sink from discovered SDP */
aes67_session_add_sink(session, "My Sink", remote_sdp_text, &id);
```

## Data Flow

```
Capture path:
  ES8311 -> I2S RX DMA -> capture ring buffer -> RTP TX -> network

Playback path:
  network -> RTP RX -> jitter buffer -> playback ring buffer -> I2S TX DMA -> ES8311

Timing:
  PTP clock -> audio frame callback (TIC) -> RTP timestamp generation
```
