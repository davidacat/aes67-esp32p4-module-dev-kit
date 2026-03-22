/*
 * SAP (Session Announcement Protocol) - RFC 2974
 *
 * Handles periodic multicast announcement and discovery of AES67
 * audio sessions. Each source stream is advertised with its SDP.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SAP multicast address and port per RFC 2974 */
#define AES67_SAP_MCAST_ADDR   "224.2.127.254"
#define AES67_SAP_PORT         9875
#define AES67_SAP_MAX_SDP_LEN  1024

/* Discovered remote source */
typedef struct {
    uint32_t origin_ip;                 /* IP of announcing node */
    uint16_t msg_id;                    /* SAP message ID hash */
    char name[64];                      /* Session name from SDP */
    char sdp[AES67_SAP_MAX_SDP_LEN];   /* Full SDP text */
    uint32_t last_seen;                 /* Tick count of last announcement */
} aes67_sap_remote_source_t;

/* Callback when a remote source is discovered or lost */
typedef void (*aes67_sap_event_cb_t)(bool is_announce,
                                     const aes67_sap_remote_source_t *source,
                                     void *user_data);

/* Opaque SAP context */
typedef struct aes67_sap_ctx *aes67_sap_handle_t;

/**
 * Initialize the SAP subsystem.
 */
esp_err_t aes67_sap_init(aes67_sap_handle_t *handle);

/**
 * Start SAP announcements and listening.
 */
esp_err_t aes67_sap_start(aes67_sap_handle_t handle);

/**
 * Stop SAP.
 */
esp_err_t aes67_sap_stop(aes67_sap_handle_t handle);

/**
 * Destroy SAP context.
 */
esp_err_t aes67_sap_destroy(aes67_sap_handle_t handle);

/**
 * Announce a local source. The SDP will be periodically multicast.
 */
esp_err_t aes67_sap_announce(aes67_sap_handle_t handle,
                             uint16_t msg_id, uint32_t origin_ip,
                             const char *sdp);

/**
 * Send a deletion for a previously announced source.
 */
esp_err_t aes67_sap_delete(aes67_sap_handle_t handle,
                           uint16_t msg_id, uint32_t origin_ip,
                           const char *sdp);

/**
 * Register callback for remote source discovery/loss events.
 */
esp_err_t aes67_sap_register_cb(aes67_sap_handle_t handle,
                                aes67_sap_event_cb_t cb, void *user_data);

/**
 * Get the number of currently known remote sources.
 */
int aes67_sap_get_remote_count(aes67_sap_handle_t handle);

/**
 * Get a remote source by index (0-based). Returns ESP_ERR_NOT_FOUND if
 * the index is out of range.
 */
esp_err_t aes67_sap_get_remote(aes67_sap_handle_t handle, int index,
                               aes67_sap_remote_source_t *source);

#ifdef __cplusplus
}
#endif
