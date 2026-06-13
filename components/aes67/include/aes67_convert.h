/*
 * SPDX-FileCopyrightText: 2026 DatanoiseTV
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * Audio sample format conversion routines.
 *
 * Based on RAVENNA MTConvert pattern. Handles conversion between native
 * (little-endian, 32-bit left-justified) audio buffers and network
 * (big-endian, packed) RTP payload formats.
 *
 * Function pointer types allow per-stream codec dispatch without
 * branching in the hot path.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Function pointer types (MTConvert pattern) --- */

/* Convert interleaved int32 native samples to packed big-endian wire format.
 * Called once per TX packet. Set at stream creation based on codec. */
typedef void (*aes67_convert_to_net_fn)(const int32_t *src, uint8_t *dst,
                                        uint32_t frame_count, uint8_t channels);

/* Convert packed big-endian wire format to interleaved int32 native samples.
 * Called once per RX packet. Set at stream creation based on codec. */
typedef void (*aes67_convert_from_net_fn)(const uint8_t *src, int32_t *dst,
                                          uint32_t frame_count, uint8_t channels);

/* Resolve the appropriate conversion functions for a given codec/word_length */
aes67_convert_to_net_fn aes67_get_to_net_fn(uint8_t word_length);
aes67_convert_from_net_fn aes67_get_from_net_fn(uint8_t word_length);

/* --- Per-codec conversion functions --- */

/* L16: 16-bit linear PCM (2 bytes/sample on wire) */
void aes67_convert_to_l16(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels);
void aes67_convert_from_l16(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels);

/* L24: 24-bit linear PCM (3 bytes/sample on wire) */
void aes67_convert_to_l24(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels);
void aes67_convert_from_l24(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels);

/* L32: 32-bit linear PCM (4 bytes/sample on wire) */
void aes67_convert_to_l32(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels);
void aes67_convert_from_l32(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels);

/* AM824: IEC 61937 encapsulated in 4-byte words.
 * Same wire format as L32 but with channel status and validity bits
 * in the lower 8 bits. For basic PCM passthrough, the conversion is
 * identical to L32. */
void aes67_convert_to_am824(const int32_t *src, uint8_t *dst,
                             uint32_t frame_count, uint8_t channels);
void aes67_convert_from_am824(const uint8_t *src, int32_t *dst,
                               uint32_t frame_count, uint8_t channels);

/* Generic dispatch (uses switch internally -- prefer function pointers
 * in hot paths for zero-branch overhead) */
void aes67_convert_to_net(const int32_t *src, uint8_t *dst,
                          uint32_t frame_count, uint8_t channels,
                          uint8_t word_length);
void aes67_convert_from_net(const uint8_t *src, int32_t *dst,
                            uint32_t frame_count, uint8_t channels,
                            uint8_t word_length);

#ifdef __cplusplus
}
#endif
