/*
 * Audio I/O driver using I2S TDM on ESP32-P4.
 *
 * Manages I2S DMA transfers for audio capture and playback. Provides
 * the audio frame callback (TIC) that drives the RTP engine's timing.
 * DMA buffers are in internal SRAM; larger ring buffers use PSRAM.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "aes67_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Audio frame callback (TIC). Invoked once per audio frame period.
 * The callback should read from sink jitter buffers and write to
 * source ring buffers. */
typedef void (*aes67_audio_frame_cb_t)(uint32_t frame_count, void *user_data);

/* Opaque audio driver handle */
typedef struct aes67_audio_ctx *aes67_audio_handle_t;

/**
 * Initialize the I2S audio driver.
 *
 * @param audio_config  Audio parameters (sample rate, channels, format)
 * @param pins          I2S pin configuration
 * @param use_psram     If true, allocate ring buffers in PSRAM
 * @param ring_buf_ms   Ring buffer size in milliseconds
 * @param handle        Output handle
 */
esp_err_t aes67_audio_init(const aes67_audio_config_t *audio_config,
                           const aes67_i2s_pins_t *pins,
                           bool use_psram,
                           uint16_t ring_buf_ms,
                           aes67_audio_handle_t *handle);

/**
 * Start audio I/O (enables I2S DMA).
 */
esp_err_t aes67_audio_start(aes67_audio_handle_t handle);

/**
 * Stop audio I/O.
 */
esp_err_t aes67_audio_stop(aes67_audio_handle_t handle);

/**
 * Destroy audio driver and free all buffers.
 */
esp_err_t aes67_audio_destroy(aes67_audio_handle_t handle);

/**
 * Register the audio frame callback. The callback is invoked from a
 * high-priority task each time a DMA transfer completes (one frame period).
 */
esp_err_t aes67_audio_register_frame_cb(aes67_audio_handle_t handle,
                                        aes67_audio_frame_cb_t cb,
                                        void *user_data);

/**
 * Write samples to the playback ring buffer (sink path).
 * Called from the RTP jitter buffer read path.
 *
 * @param handle        Audio driver handle
 * @param samples       Interleaved audio samples
 * @param frame_count   Number of frames to write
 * @return ESP_OK, or ESP_ERR_NO_MEM if buffer full
 */
esp_err_t aes67_audio_write_playback(aes67_audio_handle_t handle,
                                     const void *samples, uint32_t frame_count);

/**
 * Read samples from the capture ring buffer (source path).
 * Called by the RTP engine to get audio data for transmission.
 *
 * @param handle        Audio driver handle
 * @param samples       Output buffer for interleaved samples
 * @param frame_count   Number of frames to read
 * @return ESP_OK, or ESP_ERR_NOT_FOUND if insufficient data
 */
esp_err_t aes67_audio_read_capture(aes67_audio_handle_t handle,
                                   void *samples, uint32_t frame_count);

/**
 * Get current audio parameters.
 */
esp_err_t aes67_audio_get_config(aes67_audio_handle_t handle,
                                 aes67_audio_config_t *config);

/**
 * Get buffer fill levels (for monitoring).
 *
 * @param handle            Audio driver handle
 * @param capture_frames    Frames available in capture ring buffer
 * @param playback_frames   Frames available in playback ring buffer
 */
esp_err_t aes67_audio_get_buffer_levels(aes67_audio_handle_t handle,
                                        uint32_t *capture_frames,
                                        uint32_t *playback_frames);

#ifdef __cplusplus
}
#endif
