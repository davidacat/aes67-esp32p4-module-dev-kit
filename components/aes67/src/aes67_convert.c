/*
 * Audio sample format conversion routines.
 *
 * Ported from RAVENNA MTConvert. All conversions operate on interleaved
 * audio data, converting between native little-endian 32-bit aligned
 * samples and big-endian packed network (RTP payload) format.
 */

#include "aes67_convert.h"
#include <string.h>

/* Byte swap helpers - ESP32-P4 is little-endian */
static inline uint16_t bswap16(uint16_t v)
{
    return __builtin_bswap16(v);
}

static inline uint32_t bswap32(uint32_t v)
{
    return __builtin_bswap32(v);
}

void aes67_convert_to_l16(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;
    uint16_t *out = (uint16_t *)dst;

    for (uint32_t i = 0; i < total_samples; i++) {
        /* Take upper 16 bits of 32-bit sample, convert to big-endian */
        int16_t val = (int16_t)(src[i] >> 16);
        out[i] = bswap16((uint16_t)val);
    }
}

void aes67_convert_from_l16(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;
    const uint16_t *in = (const uint16_t *)src;

    for (uint32_t i = 0; i < total_samples; i++) {
        /* Convert from big-endian, left-justify into 32-bit */
        int16_t val = (int16_t)bswap16(in[i]);
        dst[i] = ((int32_t)val) << 16;
    }
}

void aes67_convert_to_l24(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;

    for (uint32_t i = 0; i < total_samples; i++) {
        /* Take upper 24 bits, pack as 3 bytes big-endian */
        int32_t val = src[i];
        dst[i * 3 + 0] = (uint8_t)(val >> 24);     /* MSB */
        dst[i * 3 + 1] = (uint8_t)(val >> 16);
        dst[i * 3 + 2] = (uint8_t)(val >> 8);       /* LSB of 24-bit */
    }
}

void aes67_convert_from_l24(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;

    for (uint32_t i = 0; i < total_samples; i++) {
        /* Unpack 3 bytes big-endian, left-justify into 32-bit */
        int32_t val = ((int32_t)(int8_t)src[i * 3 + 0]) << 24;  /* sign-extend MSB */
        val |= ((uint32_t)src[i * 3 + 1]) << 16;
        val |= ((uint32_t)src[i * 3 + 2]) << 8;
        dst[i] = val;
    }
}

void aes67_convert_to_l32(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;
    uint32_t *out = (uint32_t *)dst;

    for (uint32_t i = 0; i < total_samples; i++) {
        out[i] = bswap32((uint32_t)src[i]);
    }
}

void aes67_convert_from_l32(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels)
{
    uint32_t total_samples = frame_count * channels;
    const uint32_t *in = (const uint32_t *)src;

    for (uint32_t i = 0; i < total_samples; i++) {
        dst[i] = (int32_t)bswap32(in[i]);
    }
}

void aes67_convert_to_net(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels,
                          uint8_t word_length)
{
    switch (word_length) {
    case 2:
        aes67_convert_to_l16(src, dst, frame_count, channels);
        break;
    case 3:
        aes67_convert_to_l24(src, dst, frame_count, channels);
        break;
    case 4:
        aes67_convert_to_l32(src, dst, frame_count, channels);
        break;
    default:
        /* Fallback to L24 */
        aes67_convert_to_l24(src, dst, frame_count, channels);
        break;
    }
}

void aes67_convert_from_net(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels,
                            uint8_t word_length)
{
    switch (word_length) {
    case 2:
        aes67_convert_from_l16(src, dst, frame_count, channels);
        break;
    case 3:
        aes67_convert_from_l24(src, dst, frame_count, channels);
        break;
    case 4:
        aes67_convert_from_l32(src, dst, frame_count, channels);
        break;
    default:
        aes67_convert_from_l24(src, dst, frame_count, channels);
        break;
    }
}
