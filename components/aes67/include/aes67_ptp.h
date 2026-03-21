/*
 * PTP (IEEE 1588) clock synchronization interface.
 *
 * Wraps ESP-IDF's PTP daemon and hardware timestamping on ESP32-P4 EMAC.
 * Provides clock status, grand master tracking, and the sample audio clock
 * (SAC) used for RTP timestamp generation.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_eth.h"
#include "aes67_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PTP lock states matching RAVENNA conventions */
typedef enum {
    AES67_PTP_UNLOCKED = 0,
    AES67_PTP_LOCKING  = 1,
    AES67_PTP_LOCKED   = 2,
} aes67_ptp_lock_state_t;

/* PTP status information */
typedef struct {
    aes67_ptp_lock_state_t lock_state;
    uint8_t grandmaster_id[8];          /* EUI-64 grand master identity */
    uint8_t domain;
    int32_t offset_ns;                  /* Current offset from master in ns */
    int32_t path_delay_ns;              /* Mean path delay in ns */
    int32_t jitter_ns;                  /* Clock jitter estimate in ns */
} aes67_ptp_status_t;

/* Opaque PTP context */
typedef struct aes67_ptp_ctx *aes67_ptp_handle_t;

/* Callback for PTP lock state changes */
typedef void (*aes67_ptp_lock_cb_t)(aes67_ptp_lock_state_t state, void *user_data);

/**
 * Initialize the PTP subsystem. Uses ESP-IDF ptpd and hardware timestamping.
 */
esp_err_t aes67_ptp_init(esp_eth_handle_t eth_handle, const aes67_ptp_config_t *config,
                         aes67_ptp_handle_t *handle);

/**
 * Start PTP synchronization.
 */
esp_err_t aes67_ptp_start(aes67_ptp_handle_t handle);

/**
 * Stop PTP synchronization.
 */
esp_err_t aes67_ptp_stop(aes67_ptp_handle_t handle);

/**
 * Destroy the PTP context.
 */
esp_err_t aes67_ptp_destroy(aes67_ptp_handle_t handle);

/**
 * Get current PTP status.
 */
esp_err_t aes67_ptp_get_status(aes67_ptp_handle_t handle, aes67_ptp_status_t *status);

/**
 * Get the current PTP time as a 64-bit nanosecond value.
 */
esp_err_t aes67_ptp_get_time_ns(aes67_ptp_handle_t handle, uint64_t *time_ns);

/**
 * Get the current PTP time as seconds + nanoseconds.
 */
esp_err_t aes67_ptp_get_time(aes67_ptp_handle_t handle, uint32_t *sec, uint32_t *nsec);

/**
 * Convert PTP time to an RTP media timestamp at the given sample rate.
 */
uint32_t aes67_ptp_time_to_rtp_ts(uint64_t ptp_time_ns, uint32_t sample_rate);

/**
 * Register a callback for PTP lock state transitions.
 */
esp_err_t aes67_ptp_register_lock_cb(aes67_ptp_handle_t handle,
                                     aes67_ptp_lock_cb_t cb, void *user_data);

/**
 * Get the grand master ID formatted as a string (e.g. "00-1A-2B-FF-FE-3C-4D-5E").
 */
esp_err_t aes67_ptp_get_grandmaster_str(aes67_ptp_handle_t handle, char *buf, size_t len);

#ifdef __cplusplus
}
#endif
