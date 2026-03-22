/*
 * L2 TAP based RTP receiver for AES67.
 *
 * Bypasses the lwIP stack entirely by reading raw Ethernet frames via
 * the ESP-IDF L2 TAP virtual filesystem interface. Filters for IPv4/UDP
 * multicast RTP packets and writes decoded audio directly to I2S.
 */

#pragma once

#include <stdint.h>
#include "driver/i2s_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the L2 TAP RTP receiver task.
 *
 * Opens /dev/net/tap on the ETH_0 interface, spawns a high-priority
 * task on core 0 that reads raw Ethernet frames, filters for multicast
 * RTP on the given port, converts L24 audio to int16 mono, and writes
 * directly to the I2S TX channel with DMA pacing.
 *
 * @param tx_chan   I2S TX channel handle for audio output
 * @param rtp_port  UDP destination port to filter (typically 5004)
 */
void aes67_l2tap_rx_start(i2s_chan_handle_t tx_chan, uint16_t rtp_port);

#ifdef __cplusplus
}
#endif
