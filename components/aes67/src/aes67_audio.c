/*
 * I2S audio driver for AES67 on ESP32-P4.
 *
 * Uses ESP-IDF I2S standard mode with DMA for audio capture and playback.
 * Ring buffers bridge between I2S DMA transfers and the RTP engine.
 * DMA buffers reside in internal SRAM; ring buffers may use PSRAM.
 */

#include "aes67_audio.h"
#include <string.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"

static const char *TAG = "aes67_audio";

/* I/O task stack size and priority */
#define AUDIO_IO_TASK_STACK     (4096)
#define AUDIO_IO_TASK_PRIORITY  (22)

/* I2S DMA descriptor count. 16 descriptors at 192 frames each = 3072 frames
 * = 64ms of buffer at 48kHz. Large buffer reduces per-write overhead. */
#define DMA_DESC_NUM            16

/* Timeout for I2S read/write operations (ms) */
#define I2S_IO_TIMEOUT_MS       100

struct aes67_audio_ctx {
    i2s_chan_handle_t tx_chan;
    i2s_chan_handle_t rx_chan;
    aes67_audio_config_t config;

    /* Ring buffers (PSRAM when use_psram is set) */
    int32_t *capture_buf;
    int32_t *playback_buf;
    uint32_t buf_size_frames;
    volatile uint32_t capture_wr;
    volatile uint32_t capture_rd;
    volatile uint32_t playback_wr;
    volatile uint32_t playback_rd;

    /* DMA staging buffers (always internal SRAM) */
    uint8_t *dma_rx_buf;
    uint8_t *dma_tx_buf;
    uint32_t dma_buf_size_bytes;

    /* Frame callback */
    aes67_audio_frame_cb_t frame_cb;
    void *frame_cb_user_data;

    /* I/O task */
    TaskHandle_t io_task;
    bool running;
    bool use_psram;

    /* Samples per frame period: sample_rate * packet_time_us / 1000000 */
    uint32_t frame_size;
};

/* --- Helpers --------------------------------------------------------------- */

static i2s_data_bit_width_t word_length_to_bits(uint8_t word_length)
{
    switch (word_length) {
    case 2:  return I2S_DATA_BIT_WIDTH_16BIT;
    case 3:  return I2S_DATA_BIT_WIDTH_24BIT;
    case 4:  return I2S_DATA_BIT_WIDTH_32BIT;
    default: return I2S_DATA_BIT_WIDTH_24BIT;
    }
}

/* In ESP32-P4 Philips I2S mode with 32-bit slot width, the DMA data
 * is already left-justified (MSB-first) in the 32-bit word. This means
 * the I2S DMA format matches our internal left-justified int32 format
 * directly. No bit shifting is needed regardless of the configured
 * data bit width (16/24/32), because the I2S hardware handles the
 * slot-to-wire packing. */

/* Available frames in a ring buffer (single-producer / single-consumer safe) */
static inline uint32_t ring_available(uint32_t wr, uint32_t rd, uint32_t capacity)
{
    return (wr - rd) % capacity;
}

/* Free frames in a ring buffer */
static inline uint32_t ring_free(uint32_t wr, uint32_t rd, uint32_t capacity)
{
    /* Reserve one frame to distinguish full from empty */
    return capacity - 1 - ring_available(wr, rd, capacity);
}

/* --- DMA <-> Ring Buffer Conversion --------------------------------------- */

/* Copy DMA buffer (32-bit slots) directly to capture ring buffer.
 * Both DMA and ring buffer use left-justified int32 format. */
static void dma_to_capture_ring(struct aes67_audio_ctx *ctx,
                                const uint8_t *dma_buf,
                                uint32_t frame_count)
{
    const uint8_t ch = ctx->config.channels;
    const uint32_t cap = ctx->buf_size_frames;
    const int32_t *src = (const int32_t *)dma_buf;
    uint32_t wr = ctx->capture_wr;

    for (uint32_t f = 0; f < frame_count; f++) {
        uint32_t ring_idx = (wr % cap) * ch;
        for (uint8_t c = 0; c < ch; c++) {
            ctx->capture_buf[ring_idx + c] = src[f * ch + c];
        }
        wr++;
    }
    ctx->capture_wr = wr;
}

/* Read from playback ring buffer and copy to DMA TX buffer.
 * Both use left-justified int32 format - direct copy, no shifting. */
static void playback_ring_to_dma(struct aes67_audio_ctx *ctx,
                                 uint8_t *dma_buf,
                                 uint32_t frame_count)
{
    const uint8_t ch = ctx->config.channels;
    const uint32_t cap = ctx->buf_size_frames;
    int32_t *dst = (int32_t *)dma_buf;
    uint32_t rd = ctx->playback_rd;

    uint32_t avail = ring_available(ctx->playback_wr, rd, cap);

    for (uint32_t f = 0; f < frame_count; f++) {
        if (f < avail) {
            uint32_t ring_idx = (rd % cap) * ch;
            for (uint8_t c = 0; c < ch; c++) {
                dst[f * ch + c] = ctx->playback_buf[ring_idx + c];
            }
            rd++;
        } else {
            for (uint8_t c = 0; c < ch; c++) {
                dst[f * ch + c] = 0;
            }
        }
    }
    ctx->playback_rd = rd;
}

/* --- I/O Task ------------------------------------------------------------- */

static void audio_io_task(void *arg)
{
    struct aes67_audio_ctx *ctx = (struct aes67_audio_ctx *)arg;
    const uint32_t frame_size = ctx->frame_size;
    size_t bytes_read = 0;
    size_t bytes_written = 0;
    uint32_t frame_counter = 0;

    ESP_LOGI(TAG, "I/O task started, frame_size=%lu samples", (unsigned long)frame_size);

    while (ctx->running) {
        /* Capture is not needed for playback-only mode.
         * Just sleep to avoid busy-looping. When capture is needed
         * (for source streams from mic), this will be re-enabled. */
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;

        /* 1. Read captured audio from I2S RX DMA */
        esp_err_t ret = i2s_channel_read(ctx->rx_chan, ctx->dma_rx_buf,
                                          ctx->dma_buf_size_bytes,
                                          &bytes_read,
                                          pdMS_TO_TICKS(I2S_IO_TIMEOUT_MS));
        if (ret != ESP_OK) {
            if (ret != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "i2s_channel_read error: %s", esp_err_to_name(ret));
            }
            continue;
        }

        /* Determine how many complete frames we actually received */
        uint32_t samples_per_frame = ctx->config.channels;
        uint32_t bytes_per_frame = samples_per_frame * sizeof(int32_t);
        uint32_t frames_read = bytes_read / bytes_per_frame;

        /* 2. Convert DMA data and push into capture ring buffer */
        uint32_t cap_free = ring_free(ctx->capture_wr, ctx->capture_rd,
                                      ctx->buf_size_frames);
        uint32_t frames_to_store = (frames_read < cap_free) ? frames_read : cap_free;
        if (frames_to_store > 0) {
            dma_to_capture_ring(ctx, ctx->dma_rx_buf, frames_to_store);
        }
        if (frames_to_store < frames_read) {
            ESP_LOGD(TAG, "Capture ring overflow, dropped %lu frames",
                     (unsigned long)(frames_read - frames_to_store));
        }

        frame_counter++;

        /* 3. Invoke the frame callback (drives RTP TX path) */
        if (ctx->frame_cb) {
            ctx->frame_cb(frame_counter, ctx->frame_cb_user_data);
        }

        /* Playback (I2S TX) is now handled directly by the TIC task
         * via aes67_audio_direct_write(). We do NOT write to I2S TX
         * here to avoid two tasks competing on the same channel. */
    }

    ESP_LOGI(TAG, "I/O task exiting");
    vTaskDelete(NULL);
}

/* --- Public API ----------------------------------------------------------- */

esp_err_t aes67_audio_init(const aes67_audio_config_t *audio_config,
                           const aes67_i2s_pins_t *pins,
                           bool use_psram,
                           uint16_t ring_buf_ms,
                           aes67_audio_handle_t *handle)
{
    if (!audio_config || !pins || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    struct aes67_audio_ctx *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) {
        ESP_LOGE(TAG, "Failed to allocate audio context");
        return ESP_ERR_NO_MEM;
    }

    memcpy(&ctx->config, audio_config, sizeof(aes67_audio_config_t));
    ctx->use_psram = use_psram;

    /* Compute frame size: samples per packet period */
    ctx->frame_size = (uint32_t)audio_config->sample_rate *
                      audio_config->packet_time_us / 1000000;
    if (ctx->frame_size == 0) {
        ESP_LOGE(TAG, "Invalid sample_rate/packet_time combination");
        free(ctx);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Init: %lu Hz, %u ch, %u bytes/sample, frame_size=%lu",
             (unsigned long)audio_config->sample_rate,
             audio_config->channels,
             audio_config->word_length,
             (unsigned long)ctx->frame_size);

    /* --- Allocate ring buffers --- */
    uint32_t ring_frames = (audio_config->sample_rate * ring_buf_ms) / 1000;
    if (ring_frames < ctx->frame_size * 4) {
        ring_frames = ctx->frame_size * 4;
    }
    /* Add 1 for the sentinel frame (full/empty disambiguation) */
    ring_frames += 1;
    ctx->buf_size_frames = ring_frames;

    size_t ring_bytes = ring_frames * audio_config->channels * sizeof(int32_t);
    uint32_t alloc_caps = use_psram ? MALLOC_CAP_SPIRAM : MALLOC_CAP_DEFAULT;

    ctx->capture_buf = heap_caps_calloc(1, ring_bytes, alloc_caps);
    ctx->playback_buf = heap_caps_calloc(1, ring_bytes, alloc_caps);
    if (!ctx->capture_buf || !ctx->playback_buf) {
        ESP_LOGE(TAG, "Failed to allocate ring buffers (%u bytes each)", (unsigned)ring_bytes);
        goto err_free_bufs;
    }

    /* --- Allocate DMA staging buffers (internal SRAM) --- */
    ctx->dma_buf_size_bytes = ctx->frame_size * audio_config->channels * sizeof(int32_t);
    ctx->dma_rx_buf = heap_caps_calloc(1, ctx->dma_buf_size_bytes,
                                       MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    ctx->dma_tx_buf = heap_caps_calloc(1, ctx->dma_buf_size_bytes,
                                       MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!ctx->dma_rx_buf || !ctx->dma_tx_buf) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffers (%lu bytes each)",
                 (unsigned long)ctx->dma_buf_size_bytes);
        goto err_free_bufs;
    }

    /* --- Configure I2S channels --- */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_DESC_NUM;
    chan_cfg.dma_frame_num = 192;  /* Match playback write chunk size */
    chan_cfg.auto_clear = true;    /* Clear DMA buffer on underflow (silence) */

    esp_err_t ret = i2s_new_channel(&chan_cfg, &ctx->tx_chan, &ctx->rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(ret));
        goto err_free_bufs;
    }

    /* Map word_length to I2S bit depth */
    i2s_data_bit_width_t bit_width = word_length_to_bits(audio_config->word_length);
    i2s_slot_mode_t slot_mode = (audio_config->channels <= 2)
                                    ? I2S_SLOT_MODE_STEREO
                                    : I2S_SLOT_MODE_STEREO; /* TDM handled separately if needed */

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(audio_config->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bit_width, slot_mode),
    };

    /* For 24-bit audio the MCLK multiple must be divisible by 3.
     * The default (256) is not, so use 384 = 256 * 1.5 which works
     * for both 16-bit and 24-bit. */
    if (audio_config->word_length == 3) {
        std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;
    }

    std_cfg.gpio_cfg = (i2s_std_gpio_config_t){
        .mclk = pins->mck_gpio,
        .bclk = pins->bck_gpio,
        .ws   = pins->ws_gpio,
        .dout = pins->dout_gpio,
        .din  = pins->din_gpio,
        .invert_flags = {
            .mclk_inv = false,
            .bclk_inv = false,
            .ws_inv   = false,
        },
    };

    ret = i2s_channel_init_std_mode(ctx->tx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode TX failed: %s", esp_err_to_name(ret));
        goto err_del_channel;
    }

    ret = i2s_channel_init_std_mode(ctx->rx_chan, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode RX failed: %s", esp_err_to_name(ret));
        goto err_del_channel;
    }

    ctx->running = false;
    ctx->capture_wr = 0;
    ctx->capture_rd = 0;
    ctx->playback_wr = 0;
    ctx->playback_rd = 0;

    *handle = ctx;
    ESP_LOGI(TAG, "Audio driver initialized (ring=%lu frames, dma=%lu bytes)",
             (unsigned long)ring_frames, (unsigned long)ctx->dma_buf_size_bytes);
    return ESP_OK;

err_del_channel:
    i2s_del_channel(ctx->tx_chan);
    i2s_del_channel(ctx->rx_chan);
err_free_bufs:
    heap_caps_free(ctx->capture_buf);
    heap_caps_free(ctx->playback_buf);
    heap_caps_free(ctx->dma_rx_buf);
    heap_caps_free(ctx->dma_tx_buf);
    free(ctx);
    return ESP_ERR_NO_MEM;
}

esp_err_t aes67_audio_start(aes67_audio_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->running) {
        ESP_LOGW(TAG, "Audio already running");
        return ESP_OK;
    }

    /* Enable I2S channels */
    esp_err_t ret = i2s_channel_enable(handle->rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_enable(handle->tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable TX channel: %s", esp_err_to_name(ret));
        i2s_channel_disable(handle->rx_chan);
        return ret;
    }

    handle->running = true;

    /* Launch the I/O task on the highest priority */
    BaseType_t xret = xTaskCreatePinnedToCore(
        audio_io_task, "aes67_io",
        AUDIO_IO_TASK_STACK, handle,
        AUDIO_IO_TASK_PRIORITY, &handle->io_task,
        1  /* Pin to core 1 to keep core 0 for networking */
    );
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create I/O task");
        handle->running = false;
        i2s_channel_disable(handle->tx_chan);
        i2s_channel_disable(handle->rx_chan);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Audio I/O started");
    return ESP_OK;
}

esp_err_t aes67_audio_stop(aes67_audio_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!handle->running) {
        return ESP_OK;
    }

    /* Signal the I/O task to stop and wait for it to exit */
    handle->running = false;

    /* Give the task time to notice the flag and exit.
     * The task blocks in i2s_channel_read with I2S_IO_TIMEOUT_MS. */
    vTaskDelay(pdMS_TO_TICKS(I2S_IO_TIMEOUT_MS + 50));
    handle->io_task = NULL;

    /* Disable I2S channels */
    i2s_channel_disable(handle->tx_chan);
    i2s_channel_disable(handle->rx_chan);

    ESP_LOGI(TAG, "Audio I/O stopped");
    return ESP_OK;
}

esp_err_t aes67_audio_destroy(aes67_audio_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Stop if still running */
    if (handle->running) {
        aes67_audio_stop(handle);
    }

    /* Delete I2S channels */
    i2s_del_channel(handle->tx_chan);
    i2s_del_channel(handle->rx_chan);

    /* Free all buffers */
    heap_caps_free(handle->capture_buf);
    heap_caps_free(handle->playback_buf);
    heap_caps_free(handle->dma_rx_buf);
    heap_caps_free(handle->dma_tx_buf);

    free(handle);
    ESP_LOGI(TAG, "Audio driver destroyed");
    return ESP_OK;
}

esp_err_t aes67_audio_register_frame_cb(aes67_audio_handle_t handle,
                                        aes67_audio_frame_cb_t cb,
                                        void *user_data)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->frame_cb = cb;
    handle->frame_cb_user_data = user_data;
    return ESP_OK;
}

esp_err_t aes67_audio_write_playback(aes67_audio_handle_t handle,
                                     const void *samples, uint32_t frame_count)
{
    if (!handle || !samples) {
        return ESP_ERR_INVALID_ARG;
    }

    const int32_t *src = (const int32_t *)samples;
    const uint8_t ch = handle->config.channels;
    const uint32_t cap = handle->buf_size_frames;

    uint32_t free_frames = ring_free(handle->playback_wr, handle->playback_rd, cap);
    if (frame_count > free_frames) {
        /* Not enough room. Just overwrite - the I2S read task will
         * pick up the latest data. Don't touch playback_rd here as
         * that would race with the I2S I/O task reading it. */
        frame_count = free_frames;
        if (frame_count == 0) return ESP_OK;
    }

    uint32_t wr = handle->playback_wr;
    for (uint32_t f = 0; f < frame_count; f++) {
        uint32_t ring_idx = (wr % cap) * ch;
        for (uint8_t c = 0; c < ch; c++) {
            handle->playback_buf[ring_idx + c] = src[f * ch + c];
        }
        wr++;
    }
    handle->playback_wr = wr;

    return ESP_OK;
}

esp_err_t aes67_audio_read_capture(aes67_audio_handle_t handle,
                                   void *samples, uint32_t frame_count)
{
    if (!handle || !samples) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t *dst = (int32_t *)samples;
    const uint8_t ch = handle->config.channels;
    const uint32_t cap = handle->buf_size_frames;

    uint32_t avail = ring_available(handle->capture_wr, handle->capture_rd, cap);
    if (frame_count > avail) {
        return ESP_ERR_NOT_FOUND;
    }

    uint32_t rd = handle->capture_rd;
    for (uint32_t f = 0; f < frame_count; f++) {
        uint32_t ring_idx = (rd % cap) * ch;
        for (uint8_t c = 0; c < ch; c++) {
            dst[f * ch + c] = handle->capture_buf[ring_idx + c];
        }
        rd++;
    }
    handle->capture_rd = rd;

    return ESP_OK;
}

esp_err_t aes67_audio_direct_write(aes67_audio_handle_t handle,
                                   const int32_t *samples, uint32_t frame_count)
{
    if (!handle || !samples || frame_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Write int32 samples directly to I2S TX channel, bypassing the
     * playback ring buffer. Each int32 sample maps directly to a
     * 32-bit I2S slot in Philips mode (left-justified).
     * Use a longer timeout to ensure the DMA has time to accept data. */
    size_t bytes_written = 0;
    size_t bytes_to_write = frame_count * handle->config.channels * sizeof(int32_t);

    esp_err_t ret = i2s_channel_write(handle->tx_chan, samples, bytes_to_write,
                                       &bytes_written, pdMS_TO_TICKS(2));

    /* Log first successful write for debugging */
    static bool first_write_logged = false;
    if (!first_write_logged && bytes_written > 0) {
        first_write_logged = true;
        ESP_LOGI("aes67_audio", "First I2S direct write: %u/%u bytes, ret=%s",
                 (unsigned)bytes_written, (unsigned)bytes_to_write,
                 esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t aes67_audio_get_config(aes67_audio_handle_t handle,
                                 aes67_audio_config_t *config)
{
    if (!handle || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(config, &handle->config, sizeof(aes67_audio_config_t));
    return ESP_OK;
}

esp_err_t aes67_audio_get_buffer_levels(aes67_audio_handle_t handle,
                                        uint32_t *capture_frames,
                                        uint32_t *playback_frames)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (capture_frames) {
        *capture_frames = ring_available(handle->capture_wr, handle->capture_rd,
                                         handle->buf_size_frames);
    }
    if (playback_frames) {
        *playback_frames = ring_available(handle->playback_wr, handle->playback_rd,
                                          handle->buf_size_frames);
    }

    return ESP_OK;
}
