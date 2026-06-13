/*
 * SPDX-FileCopyrightText: 2026 DatanoiseTV
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * Hardware PTP-synchronized audio frame timer.
 *
 * Uses the EMAC target time interrupt to fire a callback at precise
 * PTP-synchronized intervals, replacing software-based timing with
 * hardware-precision audio frame boundaries.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Callback fired at each audio frame boundary from ISR context.
 * Must be IRAM_ATTR and as brief as possible - typically just
 * gives a semaphore or sets a flag. */
typedef void (*aes67_hw_timer_cb_t)(void *user_data);

/**
 * Initialize the hardware PTP frame timer.
 *
 * @param frame_period_ns  Frame period in nanoseconds (e.g. 1000000 for 1ms)
 * @param cb               Callback to fire at each frame boundary
 * @param user_data        User data passed to callback
 * @return ESP_OK on success
 */
esp_err_t aes67_hw_timer_init(uint32_t frame_period_ns,
                               aes67_hw_timer_cb_t cb, void *user_data);

/**
 * Start the hardware frame timer. Programs the first target time
 * aligned to the next PTP-time frame boundary.
 */
esp_err_t aes67_hw_timer_start(void);

/**
 * Stop the hardware frame timer.
 */
esp_err_t aes67_hw_timer_stop(void);

#ifdef __cplusplus
}
#endif
