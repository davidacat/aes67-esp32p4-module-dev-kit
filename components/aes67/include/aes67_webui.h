/*
 * AES67 Web UI - HTTP server and WebSocket backend.
 *
 * Serves an embedded SPA, exposes REST endpoints for device status
 * and stream management, and pushes real-time status updates over
 * a WebSocket connection at 2Hz.
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "aes67.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque Web UI context */
typedef struct aes67_webui *aes67_webui_handle_t;

/**
 * Initialize the Web UI server. Does not start listening yet.
 *
 * @param node   Running AES67 node instance
 * @param port   HTTP port (e.g. 80)
 * @param handle Output handle for the created context
 * @return ESP_OK on success
 */
esp_err_t aes67_webui_init(aes67_node_handle_t node, uint16_t port,
                            aes67_webui_handle_t *handle);

/**
 * Start the HTTP server and begin accepting connections.
 */
esp_err_t aes67_webui_start(aes67_webui_handle_t handle);

/**
 * Stop the HTTP server and tear down the WebSocket push task.
 */
esp_err_t aes67_webui_stop(aes67_webui_handle_t handle);

#ifdef __cplusplus
}
#endif
