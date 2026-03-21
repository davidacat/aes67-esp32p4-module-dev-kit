/*
 * Audio sample format conversion routines.
 *
 * Ported from RAVENNA MTConvert. Handles conversion between native
 * (little-endian, 32-bit aligned) audio buffers and network (big-endian,
 * packed) RTP payload formats.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Convert interleaved native samples to big-endian RTP payload (L16).
 * Input:  interleaved int32_t samples (left-justified 16-bit in 32-bit)
 * Output: big-endian interleaved int16_t
 */
void aes67_convert_to_l16(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels);

/*
 * Convert big-endian RTP payload (L16) to interleaved native samples.
 * Input:  big-endian interleaved int16_t
 * Output: interleaved int32_t samples (left-justified)
 */
void aes67_convert_from_l16(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels);

/*
 * Convert interleaved native samples to big-endian RTP payload (L24).
 * Input:  interleaved int32_t samples (left-justified 24-bit in 32-bit)
 * Output: big-endian packed 3-byte interleaved
 */
void aes67_convert_to_l24(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels);

/*
 * Convert big-endian RTP payload (L24) to interleaved native samples.
 */
void aes67_convert_from_l24(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels);

/*
 * Convert interleaved native samples to big-endian RTP payload (L32).
 * Input:  interleaved int32_t samples
 * Output: big-endian interleaved int32_t
 */
void aes67_convert_to_l32(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels);

/*
 * Convert big-endian RTP payload (L32) to interleaved native samples.
 */
void aes67_convert_from_l32(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels);

/*
 * Generic conversion from native to network format based on word length.
 * word_length: 2=L16, 3=L24, 4=L32
 */
void aes67_convert_to_net(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels,
                          uint8_t word_length);

/*
 * Generic conversion from network to native format based on word length.
 */
void aes67_convert_from_net(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels,
                            uint8_t word_length);

#ifdef __cplusplus
}
#endif
