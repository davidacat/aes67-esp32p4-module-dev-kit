# AES67/RAVENNA for ESP32-P4

An ESP-IDF component implementing the AES67 (and RAVENNA-compatible) audio-over-IP standard on the ESP32-P4, based on patterns from the [aes67-linux-daemon](https://github.com/bondagit/aes67-linux-daemon) reference implementation.

## Features

- **PTP (IEEE 1588v2)** clock synchronization using ESP32-P4 EMAC hardware timestamping
  - Slave mode: synchronizes to an external PTP grandmaster
  - Grandmaster mode: becomes PTP GM when no other master is present
  - Sub-microsecond lock detection with configurable thresholds
- **RTP audio streaming** with per-stream jitter buffers
  - Source (TX) and Sink (RX) streams
  - L16, L24, L32, AM824 codec support with MTConvert function pointer dispatch
  - Dynamic TIC sizes: 48, 64, 96, 128, 192 up to 1024 frames
  - Channels: 1-8 per stream with channel routing support
  - Sample rates: 44.1kHz, 48kHz (96kHz planned)
  - Configurable packet time (125us to 4ms, default 1ms per AES67)
  - DSCP/QoS marking for network prioritization
  - Raw lwIP UDP PCB for minimal per-packet overhead (~10us)
  - Ethernet MAC hook for zero-copy RTP interception
- **Flexible output routing** per node
  - I2S direct output to codec (DMA-paced, default)
  - User callback with int32 sample buffers
  - FreeRTOS StreamBuffer for application-level processing
- **SAP (RFC 2974)** session announcement and discovery
  - Periodic multicast announcements for source streams
  - Remote source discovery with auto-subscribe callback
- **SDP** generation and parsing (AES67-compliant)
  - PTP reference clock attributes (`a=ts-refclk`)
  - Media clock attributes (`a=mediaclk`)
  - Fractional ptime support for sub-millisecond packet times
- **mDNS/DNS-SD** service advertisement (`_ravenna._udp`)
- **I2S audio** driver with APLL clock
  - Standard I2S Philips mode, 32-bit slots
  - APLL for exact audio sample rates (18.432MHz MCLK)
  - Adaptive clock recovery via APLL SDM registers (sub-ppm tuning)
  - Ring buffers in PSRAM, DMA buffers in internal SRAM
- **Performance optimizations**
  - -O2 compiler optimization
  - IRAM_ATTR on all hot-path conversion functions
  - lwIP IRAM optimizations enabled
  - Conversion function pointers resolved at stream creation (zero-branch hot path)

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
| I2S DOUT (ESP->codec) | 9 |
| I2S DIN (codec->ESP) | 11 |
| PA Enable | 53 |

Ethernet PHY: IP101 GRI via RMII, external 50MHz crystal.
Audio codec: ES8311 via I2C + I2S, NS4150B power amplifier.

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
    aes67_config.h      Configuration structs, stream params, codec utilities
    aes67_ptp.h         PTP clock synchronization
    aes67_rtp.h         RTP packet engine
    aes67_sap.h         SAP announcement/discovery
    aes67_sdp.h         SDP parser/generator
    aes67_session.h     Session manager (sources/sinks)
    aes67_audio.h       I2S audio driver with APLL
    aes67_convert.h     Sample format conversion (MTConvert pattern)
    aes67_net.h         Network utilities
  src/
    aes67.c             Node lifecycle, playback task, output routing
    aes67_ptp.c         PTP wrapping ptpd + esp_eth_time
    aes67_rtp.c         RTP engine with ring buffers and raw UDP PCB
    aes67_sap.c         SAP multicast announcements
    aes67_sdp.c         SDP text generation/parsing
    aes67_session.c     Stream orchestration
    aes67_audio.c       I2S DMA driver with APLL clock recovery
    aes67_convert.c     L16/L24/L32/AM824 format conversion (IRAM)
    aes67_net.c         Multicast socket helpers
    aes67_mdns.c        mDNS service registration
```

## Configuration

All parameters are configurable via `idf.py menuconfig` under "AES67 RAVENNA Component":

- Max sources/sinks (default: 4 each)
- Max channels per stream (default: 8)
- Sample rate (default: 48000 Hz)
- Packet time (default: 1000us)
- Word length / codec (default: 3 = L24)
- Default channel count (default: 2)
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

### Callback output mode

```c
void my_audio_cb(const int32_t *samples, uint32_t frame_count,
                 uint8_t channels, uint8_t word_length,
                 uint32_t sample_rate, void *user_data) {
    /* Process int32 left-justified samples */
}

aes67_config_t cfg = AES67_CONFIG_DEFAULT();
cfg.output_mode = AES67_OUTPUT_CALLBACK;
cfg.audio_cb = my_audio_cb;
cfg.audio_cb_user_data = NULL;
```

## Data Flow

```
Capture path:
  ES8311 -> I2S RX DMA -> capture ring buffer -> RTP TX -> network

Playback path (I2S mode):
  network -> Ethernet hook -> stream buffer -> playback task -> I2S TX DMA -> ES8311

Playback path (Callback mode):
  network -> RTP RX -> convert -> stream buffer -> playback task -> user callback

Timing:
  PTP clock -> hardware timer ISR -> audio frame task (TIC) -> RTP TX
  APLL clock -> I2S DMA -> 48kHz sample-accurate playback
```
