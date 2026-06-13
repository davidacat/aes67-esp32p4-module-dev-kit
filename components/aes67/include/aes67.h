/*
 * SPDX-FileCopyrightText: 2026 DatanoiseTV
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * AES67/RAVENNA audio-over-IP component for ESP-IDF
 *
 * Top-level API for initialization, configuration and lifecycle management.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "aes67_config.h"
#include "aes67_session.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque handle for an AES67 node instance */
typedef struct aes67_node *aes67_node_handle_t;

/* Event types posted to the application event loop */
typedef enum {
    AES67_EVENT_PTP_LOCKED,         /* PTP clock achieved lock */
    AES67_EVENT_PTP_UNLOCKED,       /* PTP clock lost lock */
    AES67_EVENT_SOURCE_ADDED,       /* Local source stream created */
    AES67_EVENT_SOURCE_REMOVED,     /* Local source stream removed */
    AES67_EVENT_SINK_ADDED,         /* Local sink stream created */
    AES67_EVENT_SINK_REMOVED,       /* Local sink stream removed */
    AES67_EVENT_REMOTE_FOUND,       /* Remote source discovered via SAP/mDNS */
    AES67_EVENT_REMOTE_LOST,        /* Remote source timed out */
    AES67_EVENT_SINK_RX_ACTIVE,     /* Sink is receiving audio data */
    AES67_EVENT_SINK_RX_TIMEOUT,    /* Sink stopped receiving audio data */
} aes67_event_t;

/* Event base declaration for ESP event loop */
ESP_EVENT_DECLARE_BASE(AES67_EVENT);

/**
 * Initialize the AES67 node with the given configuration.
 *
 * Brings up PTP, RTP engine, SAP, mDNS and audio subsystems.
 * The Ethernet interface must already be initialized and connected.
 *
 * @param config  Node configuration. NULL uses defaults from Kconfig.
 * @param handle  Output handle for the created node instance.
 * @return ESP_OK on success
 */
esp_err_t aes67_node_init(const aes67_config_t *config, aes67_node_handle_t *handle);

/**
 * Start the AES67 node. Begins PTP sync, SAP announcements and enables
 * stream processing.
 */
esp_err_t aes67_node_start(aes67_node_handle_t handle);

/**
 * Stop the AES67 node. Tears down all active streams and halts PTP.
 */
esp_err_t aes67_node_stop(aes67_node_handle_t handle);

/**
 * Destroy the AES67 node and free all resources.
 */
esp_err_t aes67_node_destroy(aes67_node_handle_t handle);

/**
 * Get the node's current configuration (read-only snapshot).
 */
esp_err_t aes67_node_get_config(aes67_node_handle_t handle, aes67_config_t *config);

/**
 * Get the session manager handle from a running node.
 * Useful for adding/removing sources and sinks at the application level.
 */
esp_err_t aes67_node_get_session(aes67_node_handle_t handle,
                                  aes67_session_handle_t *session);

/**
 * Get the audio driver handle from a running node.
 */
#include "aes67_audio.h"
esp_err_t aes67_node_get_audio(aes67_node_handle_t handle,
                               aes67_audio_handle_t *audio);

/**
 * Get the SAP handle from a running node.
 */
#include "aes67_sap.h"
esp_err_t aes67_node_get_sap(aes67_node_handle_t handle,
                              aes67_sap_handle_t *sap);

/**
 * Get the PTP handle from a running node.
 */
#include "aes67_ptp.h"
esp_err_t aes67_node_get_ptp(aes67_node_handle_t handle,
                              aes67_ptp_handle_t *ptp);

/**
 * Get the RTP engine handle from a running node.
 */
#include "aes67_rtp.h"
esp_err_t aes67_node_get_rtp(aes67_node_handle_t handle,
                              aes67_rtp_engine_handle_t *rtp);

/**
 * Provide an external playout stream buffer for the node's playback task.
 *
 * By default the playback task consumes audio from the RTP engine's own
 * stream buffer. An integration that intercepts RTP frames earlier (for
 * example, an esp_eth Rx hook that parses audio in the driver callback and
 * feeds it in directly) can register its own buffer here; when set and
 * non-NULL, the playback task reads from it instead. Pass NULL to clear.
 *
 * Optional. Call before aes67_node_start(); has no effect on the ISR-driven
 * I2S path, which is fed via aes67_audio_slot_write().
 */
esp_err_t aes67_node_set_playout_buf(aes67_node_handle_t handle,
                                      StreamBufferHandle_t sbuf);

#ifdef __cplusplus
}
#endif
