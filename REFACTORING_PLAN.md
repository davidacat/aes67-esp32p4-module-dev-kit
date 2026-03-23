# AES67 ESP32-P4 Refactoring Plan

## Goal
Production-ready, flexible AES67 node supporting:
- Dynamic TIC sizes: 48, 64, 96, 128, 192, up to 1024 frames
- Sample rates: 44.1kHz, 48kHz (96kHz future)
- Codecs: L16, L24, L32, AM824
- Channels: 1-8 per stream
- Output: I2S / StreamBuffer / Callback
- WebUI with REST API and WebSocket

## Architecture (from linux-aes67-daemon reference)

### Internal Format
- All audio stored as **int32 left-justified** regardless of wire codec
- Conversion only at RTP packet boundaries (MTConvert pattern)
- Per-stream conversion function pointers (set at stream creation)

### Stream Parameters
Computed once at stream creation from (sample_rate, channels, word_length, packet_time_us):
- samples_per_packet = sample_rate * packet_time_us / 1000000
- frame_bytes_wire = channels * word_length
- frame_bytes_native = channels * sizeof(int32_t)
- packet_payload_size = samples_per_packet * frame_bytes_wire

### Per-Channel Jitter Buffers
- Each channel has its own ring buffer (linux uses SAC lock-free access)
- Channel routing array maps stream channels to hardware channels
- Playout delay configurable per stream

### Output Routing
- Per-stream output mode: I2S / StreamBuffer / Callback
- Allows one sink on I2S while another feeds application processing
- Channel routing for I2S (which stream channels go to which I2S slots)

## Phase 1: Core Engine Tasks

### 1.1 Stream Parameters Struct
- aes67_stream_params_t with all derived sizes
- Replace all hardcoded 192, 2, 48000, 768, 1536

### 1.2 Multi-Rate (44.1kHz + 48kHz)
- APLL reconfiguration per sample rate
- ES8311 clock coefficient table covers both rates

### 1.3 Multi-Codec (L16/L24/L32/AM824)
- Function pointers for conversion (MTConvert pattern)
- AM824 conversion implementation
- I2S always 32-bit slots (internal = int32)

### 1.4 Dynamic Channels (1-8)
- Per-channel buffers
- Channel routing array
- Remove stereo assumptions

### 1.5 Clean Module Boundaries
- Remove all extern hacks
- Proper public API headers

### 1.6 Flexible Output
- Output mode enum per stream
- I2S / StreamBuffer / Callback
- Remove Ethernet hook (duplicates RTP processing)

### 1.7 Public API
- Clean aes67.h with all user-facing functions
- Callback registration
- Stream statistics access

## Phase 2: WebUI

### 2.1 HTTP REST API (esp_http_server + cJSON)
### 2.2 WebSocket (real-time status at 2Hz)
### 2.3 Embedded SPA (vanilla HTML/CSS/JS, gzipped in flash)
### 2.4 Audio Metering (RMS/peak per channel via esp-dsp)

## Phase 3: Advanced Optimizations

### 3.1 BitScrambler
- ESP32-P4 BitScrambler attaches to I2S DMA
- Can do L24 big-endian → int32 conversion in hardware
- Zero CPU cycles for format conversion
- Program: read 24 bits, route to bits 8-31, zero bits 0-7, write 32 bits

### 3.2 Adaptive APLL Clock Recovery
- Use APLL SDM registers for sub-ppm frequency adjustment
- PI controller on stream buffer level
- Match I2S clock to network source rate

### 3.3 L2 TAP with selective forwarding
- Capture RTP frames at L2, forward rest to lwIP
- Requires per-frame forwarding logic
