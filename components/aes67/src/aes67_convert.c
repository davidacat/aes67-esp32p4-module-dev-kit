/*
 * Audio sample format conversion routines.
 *
 * Based on RAVENNA MTConvert. All conversions operate on interleaved
 * audio data, converting between native little-endian 32-bit aligned
 * samples and big-endian packed network (RTP payload) format.
 *
 * Hot-path functions are placed in IRAM to avoid flash cache misses
 * during real-time audio processing.
 */

#include "aes67_convert.h"
#include <string.h>
#include "esp_attr.h"

/* Byte swap helpers - ESP32-P4 is little-endian */
static inline uint16_t bswap16(uint16_t v)
{
    return __builtin_bswap16(v);
}

static inline uint32_t bswap32(uint32_t v)
{
    return __builtin_bswap32(v);
}

/* --- L16: 16-bit linear PCM --- */

IRAM_ATTR void aes67_convert_to_l16(const int32_t *src, uint8_t *dst,
                                     uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;
    uint16_t *out = (uint16_t *)dst;

    for (uint32_t i = 0; i < total_samples; i++) {
        int16_t val = (int16_t)(src[i] >> 16);
        out[i] = bswap16((uint16_t)val);
    }
}

IRAM_ATTR void aes67_convert_from_l16(const uint8_t *src, int32_t *dst,
                                       uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;
    const uint16_t *in = (const uint16_t *)src;

    for (uint32_t i = 0; i < total_samples; i++) {
        int16_t val = (int16_t)bswap16(in[i]);
        dst[i] = ((int32_t)val) << 16;
    }
}

/* --- L24: 24-bit linear PCM --- */

IRAM_ATTR void aes67_convert_to_l24(const int32_t *src, uint8_t *dst,
                                     uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;

    for (uint32_t i = 0; i < total_samples; i++) {
        int32_t val = src[i];
        dst[i * 3 + 0] = (uint8_t)(val >> 24);
        dst[i * 3 + 1] = (uint8_t)(val >> 16);
        dst[i * 3 + 2] = (uint8_t)(val >> 8);
    }
}

IRAM_ATTR void aes67_convert_from_l24(const uint8_t *src, int32_t *dst,
                                       uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;

    for (uint32_t i = 0; i < total_samples; i++) {
        int32_t val = ((int32_t)(int8_t)src[i * 3 + 0]) << 24;
        val |= ((uint32_t)src[i * 3 + 1]) << 16;
        val |= ((uint32_t)src[i * 3 + 2]) << 8;
        dst[i] = val;
    }
}

/* --- L32: 32-bit linear PCM --- */

IRAM_ATTR void aes67_convert_to_l32(const int32_t *src, uint8_t *dst,
                                     uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;
    uint32_t *out = (uint32_t *)dst;

    for (uint32_t i = 0; i < total_samples; i++) {
        out[i] = bswap32((uint32_t)src[i]);
    }
}

IRAM_ATTR void aes67_convert_from_l32(const uint8_t *src, int32_t *dst,
                                       uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;
    const uint32_t *in = (const uint32_t *)src;

    for (uint32_t i = 0; i < total_samples; i++) {
        dst[i] = (int32_t)bswap32(in[i]);
    }
}

/* --- AM824: IEC 61937 in 4-byte words --- */

/* For basic PCM passthrough, AM824 is byte-swapped int32 like L32.
 * The lower 8 bits carry channel status/validity but we zero them
 * on TX and ignore them on RX for PCM content. */
IRAM_ATTR void aes67_convert_to_am824(const int32_t *src, uint8_t *dst,
                                       uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;
    uint32_t *out = (uint32_t *)dst;

    for (uint32_t i = 0; i < total_samples; i++) {
        /* Pack audio into upper 24 bits, clear validity/channel status */
        out[i] = bswap32((uint32_t)(src[i] & 0xFFFFFF00));
    }
}

IRAM_ATTR void aes67_convert_from_am824(const uint8_t *src, int32_t *dst,
                                         uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;
    const uint32_t *in = (const uint32_t *)src;

    for (uint32_t i = 0; i < total_samples; i++) {
        /* Extract audio from upper 24 bits, mask off channel status */
        dst[i] = (int32_t)(bswap32(in[i]) & 0xFFFFFF00);
    }
}

/* --- Function pointer resolvers (MTConvert pattern) --- */

aes67_convert_to_net_fn aes67_get_to_net_fn(uint8_t word_length)
{
    switch (word_length) {
    case 2: return aes67_convert_to_l16;
    case 3: return aes67_convert_to_l24;
    case 4: return aes67_convert_to_l32;
    default: return aes67_convert_to_l24;
    }
}

aes67_convert_from_net_fn aes67_get_from_net_fn(uint8_t word_length)
{
    switch (word_length) {
    case 2: return aes67_convert_from_l16;
    case 3: return aes67_convert_from_l24;
    case 4: return aes67_convert_from_l32;
    default: return aes67_convert_from_l24;
    }
}

/* --- Generic dispatch (for non-hot-path use) --- */

void aes67_convert_to_net(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels,
                          uint8_t word_length)
{
    aes67_get_to_net_fn(word_length)(src, dst, frame_count, channels);
}

void aes67_convert_from_net(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels,
                            uint8_t word_length)
{
    aes67_get_from_net_fn(word_length)(src, dst, frame_count, channels);
}
