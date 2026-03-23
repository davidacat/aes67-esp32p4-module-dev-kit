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
#include "esp_timer.h"
#include "esp_cache.h"
#include "hal/i2s_ll.h"
#include "hal/hal_utils.h"
#include "soc/hp_sys_clkrst_struct.h"

static const char *TAG = "aes67_audio";

/* Kept for future capture (RX) task */

/* I2S DMA: 4 descriptors at 192 frames each = 768 frames = 16ms.
 * DMA descriptor size is independent of RTP packet time --
 * it's an I2S driver buffering parameter, not a network parameter. */
#define DMA_DESC_NUM            4
#define DMA_FRAME_NUM           192


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

    /* Pre-allocated int16 conversion buffer for direct_write.
     * Avoids heap_caps_malloc/free on every write call. */
    int16_t *i2s_conv_buf;
    uint32_t i2s_conv_buf_samples;

    /* DMA completion semaphore - signaled by on_sent ISR callback */
    SemaphoreHandle_t dma_sem;

    /* Staging ring buffer: RTP RX writes int16 data here,
     * DMA ISR reads from here to fill DMA descriptors directly. */
    int16_t *staging_buf;
    uint32_t staging_size;      /* Total size in int16 samples */
    volatile uint32_t staging_wr;
    volatile uint32_t staging_rd;
    volatile uint32_t staging_underruns;  /* ISR couldn't fill full descriptor */
    volatile uint32_t staging_isr_count;  /* Total ISR calls */
    volatile bool staging_ready;          /* ISR reads only when true */

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

/* Convert word_length (bytes per sample) to I2S bit width enum.
 * Currently unused since we force 16-bit I2S for ES8311 compat,
 * but kept for future use with other codecs. */
__attribute__((unused))
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
__attribute__((unused))
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
__attribute__((unused))
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

/* --- DMA callbacks -------------------------------------------------------- */

/* Stream buffer fed by the Ethernet hook. The DMA ISR reads from this
 * directly, bypassing the playback task entirely for zero overhead. */
static StreamBufferHandle_t s_isr_stream_buf = NULL;

/* ISR diagnostics (read by monitor task) */
static volatile uint32_t s_isr_fires = 0;
static volatile uint32_t s_isr_full = 0;     /* ISR got a full descriptor of data */
static volatile uint32_t s_isr_partial = 0;  /* ISR got some data but not full descriptor */
static volatile uint32_t s_isr_empty = 0;    /* ISR got zero bytes (silence) */

/* Pre-buffer threshold: wait until this many bytes are in the stream
 * buffer before the ISR starts consuming. This absorbs the phase
 * jitter between packet arrival (network-timed) and ISR firing
 * (APLL-timed). Without pre-buffering, the ISR and packets are two
 * independent 250Hz clocks that drift in and out of phase, causing
 * ~10% of ISR fires to find the buffer empty. */
#define ISR_PREFILL_BYTES   (1536 * 3)  /* 3 packets = 12ms headroom */
static volatile bool s_isr_prefilled = false;

/* ISR callback: fires when a DMA descriptor finishes playing (every 4ms).
 * Reads int32 audio data directly from the stream buffer into the DMA
 * descriptor. If no data is available, writes silence. */
static IRAM_ATTR bool on_i2s_tx_sent_sbuf(i2s_chan_handle_t handle,
                                            i2s_event_data_t *event,
                                            void *user_ctx)
{
    BaseType_t woken = pdFALSE;
    s_isr_fires++;

    if (!s_isr_stream_buf) {
        s_isr_empty++;
        memset(event->dma_buf, 0, event->size);
        esp_cache_msync(event->dma_buf, event->size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        return false;
    }

    /* Wait for pre-buffer to fill before starting consumption.
     * This gives 12ms of headroom so the ISR always finds data
     * even when packet arrival and ISR firing are out of phase. */
    if (!s_isr_prefilled) {
        size_t avail = xStreamBufferBytesAvailable(s_isr_stream_buf);
        if (avail < ISR_PREFILL_BYTES) {
            s_isr_empty++;
            memset(event->dma_buf, 0, event->size);
            esp_cache_msync(event->dma_buf, event->size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
            return false;
        }
        s_isr_prefilled = true;
    }

    size_t received = xStreamBufferReceiveFromISR(s_isr_stream_buf,
                                                   event->dma_buf,
                                                   event->size,
                                                   &woken);
    if (received >= event->size) {
        s_isr_full++;
    } else if (received > 0) {
        s_isr_partial++;
        memset((uint8_t *)event->dma_buf + received, 0,
               event->size - received);
        /* Buffer underrun after prefill: reset and re-prefill */
        s_isr_prefilled = false;
    } else {
        s_isr_empty++;
        memset(event->dma_buf, 0, event->size);
        s_isr_prefilled = false;  /* Re-prefill on next data */
    }

    /* Flush CPU cache so DMA sees the new data */
    esp_cache_msync(event->dma_buf, event->size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    return woken == pdTRUE;
}

/* Called periodically to log ISR stats */
void aes67_audio_log_isr_stats(void)
{
    uint32_t fires = s_isr_fires;
    uint32_t full = s_isr_full;
    uint32_t partial = s_isr_partial;
    uint32_t empty = s_isr_empty;
    s_isr_fires = 0;
    s_isr_full = 0;
    s_isr_partial = 0;
    s_isr_empty = 0;
    if (fires > 0) {
        size_t sb_bytes = s_isr_stream_buf ?
                          xStreamBufferBytesAvailable(s_isr_stream_buf) : 0;
        ESP_LOGI("dma_isr", "fires=%lu full=%lu partial=%lu empty=%lu (%lu%%) sb=%u",
                 (unsigned long)fires, (unsigned long)full,
                 (unsigned long)partial, (unsigned long)empty,
                 (unsigned long)(full * 100 / fires),
                 (unsigned)sb_bytes);
    }
}

/* Set the stream buffer for ISR-driven playback */
void aes67_audio_set_isr_stream_buf(aes67_audio_handle_t handle,
                                     StreamBufferHandle_t sbuf)
{
    (void)handle;
    s_isr_stream_buf = sbuf;
}

/* Legacy staging ISR (unused, kept for reference) */
__attribute__((unused))
static IRAM_ATTR bool on_i2s_tx_sent(i2s_chan_handle_t handle,
                                      i2s_event_data_t *event,
                                      void *user_ctx)
{
    struct aes67_audio_ctx *ctx = (struct aes67_audio_ctx *)user_ctx;
    int16_t *dma_buf = (int16_t *)event->dma_buf;
    uint32_t dma_samples = event->size / sizeof(int16_t);

    /* Don't read from staging until it has enough data */
    if (!ctx->staging_ready) {
        memset(dma_buf, 0, event->size);
        esp_cache_msync(event->dma_buf, event->size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        ctx->staging_isr_count++;
        return false;
    }

    /* Copy from staging ring to DMA buffer */
    uint32_t rd = ctx->staging_rd;
    uint32_t wr = ctx->staging_wr;
    uint32_t cap = ctx->staging_size;
    uint32_t avail = (wr >= rd) ? (wr - rd) : (cap - rd + wr);

    uint32_t to_copy = (avail < dma_samples) ? avail : dma_samples;
    uint32_t first = cap - (rd % cap);
    if (first > to_copy) first = to_copy;

    /* Fast copy from staging ring (internal SRAM, no cache issues) */
    memcpy(dma_buf, &ctx->staging_buf[rd % cap], first * sizeof(int16_t));
    if (to_copy > first) {
        memcpy(dma_buf + first, ctx->staging_buf,
               (to_copy - first) * sizeof(int16_t));
    }

    /* Zero-fill if not enough data (silence on underrun) */
    if (to_copy < dma_samples) {
        memset(dma_buf + to_copy, 0, (dma_samples - to_copy) * sizeof(int16_t));
    }

    ctx->staging_rd = (rd + to_copy) % cap;
    ctx->staging_isr_count++;
    if (to_copy < dma_samples) {
        ctx->staging_underruns++;
    }

    /* Flush CPU cache to memory so DMA controller sees the new data.
     * ESP32-P4 routes internal SRAM through L1 cache -- without this
     * sync, DMA reads stale data and produces crackling. */
    esp_cache_msync(event->dma_buf, event->size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    /* Notify playback task to refill staging buffer */
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(ctx->dma_sem, &woken);
    return woken == pdTRUE;
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
    /* Official ESP-IDF es8311 example for ESP32-P4 uses I2S_NUM_0 */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_DESC_NUM;
    chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    /* When using ISR callback, auto_clear doesn't matter because the ISR
     * fills every descriptor (with data or silence). Set false for
     * i2s_channel_write compatibility in non-ISR modes. */
    chan_cfg.auto_clear = (s_isr_stream_buf != NULL);

    esp_err_t ret = i2s_new_channel(&chan_cfg, &ctx->tx_chan, &ctx->rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(ret));
        goto err_free_bufs;
    }

    /* Map word_length to I2S bit depth */
    i2s_slot_mode_t slot_mode = I2S_SLOT_MODE_STEREO;

    /* 32-bit I2S: int32 maps directly to DMA, no conversion needed.
     * ES8311 supports 32-bit (SDP_IN_WL = 100). */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(audio_config->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, slot_mode),
    };

    /* Use APLL clock source for precise audio frequency control.
     * APLL has SDM (Sigma-Delta Modulator) for sub-ppm tuning.
     * ESP32-P4 Ethernet uses MPLL, so no APLL conflict. */
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;

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

    /* Staging ring: RTP RX writes int16, DMA ISR reads.
     * 10 packets of headroom to absorb network jitter and
     * phase differences between packet arrival and DMA consumption. */
    ctx->staging_size = ctx->frame_size * 10 * audio_config->channels;
    ctx->staging_buf = heap_caps_calloc(ctx->staging_size, sizeof(int16_t),
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!ctx->staging_buf) {
        ESP_LOGE(TAG, "Failed to allocate staging ring");
        goto err_del_channel;
    }
    ctx->staging_wr = 0;
    ctx->staging_rd = 0;
    ctx->staging_ready = false;

    /* Binary semaphore: DMA ISR signals when descriptor consumed */
    ctx->dma_sem = xSemaphoreCreateBinary();
    if (!ctx->dma_sem) {
        ESP_LOGE(TAG, "Failed to create DMA semaphore");
        goto err_del_channel;
    }

    /* Pre-allocate int16 conversion buffer for direct_write.
     * Size for max batch: 25 packets worth of frames. */
    ctx->i2s_conv_buf_samples = ctx->frame_size * 25 * audio_config->channels;
    ctx->i2s_conv_buf = heap_caps_malloc(
        ctx->i2s_conv_buf_samples * sizeof(int16_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!ctx->i2s_conv_buf) {
        ESP_LOGE(TAG, "Failed to allocate I2S conversion buffer (%lu bytes)",
                 (unsigned long)(ctx->i2s_conv_buf_samples * sizeof(int16_t)));
        goto err_del_channel;
    }

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

    /* Register DMA ISR callback for direct stream buffer -> DMA transfer.
     * This bypasses i2s_channel_write and the playback task entirely. */
    if (s_isr_stream_buf) {
        i2s_event_callbacks_t cbs = {
            .on_recv = NULL,
            .on_recv_q_ovf = NULL,
            .on_sent = on_i2s_tx_sent_sbuf,
            .on_send_q_ovf = NULL,
        };
        esp_err_t cb_ret = i2s_channel_register_event_callback(
            handle->tx_chan, &cbs, handle);
        if (cb_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to register DMA ISR callback: %s",
                     esp_err_to_name(cb_ret));
        } else {
            ESP_LOGI(TAG, "DMA ISR callback registered (zero-overhead playback)");
        }
    }

    /* Enable both TX and RX channels. The ES8311 codec requires both
     * to be active (matching the official ESP-IDF es8311 example). */
    esp_err_t ret = i2s_channel_enable(handle->tx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_enable(handle->rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enable RX channel: %s", esp_err_to_name(ret));
        /* Non-fatal: TX still works */
    }

    handle->running = true;

    /* Read and cache the clock divider set by i2s_channel_init_std_mode.
     * This gives us the correct base values for adaptive clock recovery. */
    extern void aes67_audio_read_clock_divider(aes67_audio_handle_t h);
    aes67_audio_read_clock_divider(handle);

    ESP_LOGI(TAG, "Audio started (DMA: %d desc x %d frames, DMA-paced)",
             DMA_DESC_NUM, DMA_FRAME_NUM);
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

    handle->running = false;

    /* Disable I2S channels */
    i2s_channel_disable(handle->tx_chan);
    i2s_channel_disable(handle->rx_chan);

    ESP_LOGI(TAG, "Audio stopped");
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
    heap_caps_free(handle->i2s_conv_buf);

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

    uint32_t total_samples = frame_count * handle->config.channels;

    /* Clamp to pre-allocated buffer capacity */
    if (total_samples > handle->i2s_conv_buf_samples) {
        total_samples = handle->i2s_conv_buf_samples;
        frame_count = total_samples / handle->config.channels;
    }

    /* With 24-bit I2S (32-bit slots), write int32 directly to DMA.
     * The L24 conversion already produces left-justified int32.
     * No truncation needed -- full 24-bit quality preserved. */
    size_t bytes_written = 0;
    size_t bytes_to_write = total_samples * sizeof(int32_t);

    int64_t t0 = esp_timer_get_time();
    esp_err_t ret = i2s_channel_write(handle->tx_chan, samples, bytes_to_write,
                                       &bytes_written, portMAX_DELAY);
    int64_t t1 = esp_timer_get_time();

    /* Measure i2s_channel_write timing to find the throughput bottleneck */
    static int64_t total_write_us = 0;
    static int64_t total_between_us = 0;
    static int64_t last_end_us = 0;
    if (last_end_us > 0) {
        total_between_us += (t0 - last_end_us);
    }
    last_end_us = t1;
    total_write_us += (t1 - t0);

    /* Log first few writes with sample values for debugging format issues */
    static uint32_t write_log_count = 0;
    if (write_log_count < 3) {
        write_log_count++;
        ESP_LOGI(TAG, "I2S write #%lu: %u/%u bytes, frames=%lu, ret=%s, "
                 "write=%lldus, between=%lldus | "
                 "int32[0..3]: %+ld %+ld %+ld %+ld -> "
                 "int16[0..3]: %+d %+d %+d %+d",
                 (unsigned long)write_log_count,
                 (unsigned)bytes_written, (unsigned)bytes_to_write,
                 (unsigned long)frame_count, esp_err_to_name(ret),
                 (long long)(t1 - t0),
                 (long long)(last_end_us > 0 ? (t0 - (last_end_us - (t1 - t0))) : 0),
                 (long)samples[0], (long)samples[1],
                 (long)samples[2], (long)samples[3],
                 (long)samples[0], (long)samples[1],
                 (long)samples[2], (long)samples[3]);
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

/* Write int16 stereo samples to the staging ring buffer.
 * Called by the playback task. The DMA ISR reads from this ring. */
esp_err_t aes67_audio_staging_write(aes67_audio_handle_t handle,
                                     const int16_t *samples, uint32_t sample_count)
{
    if (!handle || !samples) return ESP_ERR_INVALID_ARG;

    uint32_t cap = handle->staging_size;
    uint32_t wr = handle->staging_wr;
    uint32_t rd = handle->staging_rd;
    uint32_t avail = (wr >= rd) ? (wr - rd) : (cap - rd + wr);
    uint32_t free_samples = cap - 1 - avail;  /* -1 for sentinel */

    if (sample_count > free_samples) {
        sample_count = free_samples;
    }
    if (sample_count == 0) return ESP_OK;

    uint32_t pos = wr % cap;
    uint32_t first = cap - pos;
    if (first > sample_count) first = sample_count;

    memcpy(&handle->staging_buf[pos], samples, first * sizeof(int16_t));
    if (sample_count > first) {
        memcpy(handle->staging_buf, samples + first,
               (sample_count - first) * sizeof(int16_t));
    }

    handle->staging_wr = (wr + sample_count) % cap;

    /* Enable ISR reads once staging is at least half full (20ms).
     * More headroom = more tolerance for network jitter. */
    if (!handle->staging_ready) {
        uint32_t new_wr = handle->staging_wr;
        uint32_t cur_rd = handle->staging_rd;
        uint32_t filled = (new_wr >= cur_rd) ? (new_wr - cur_rd) : (cap - cur_rd + new_wr);
        if (filled >= cap / 2) {
            handle->staging_ready = true;
        }
    }

    return ESP_OK;
}

void aes67_audio_get_staging_stats(aes67_audio_handle_t handle,
                                    uint32_t *isr_count, uint32_t *underruns)
{
    if (handle) {
        if (isr_count) *isr_count = handle->staging_isr_count;
        if (underruns) *underruns = handle->staging_underruns;
    }
}

/* Cached base divider values (read from registers after I2S init) */
static uint32_t base_div_n = 0;
static uint32_t base_div_x = 0;
static uint32_t base_div_y = 0;
static uint32_t base_div_z = 0;
static uint32_t base_div_yn1 = 0;
static uint32_t base_numerator = 0;
static uint32_t base_denominator = 0;

/* Read and cache the I2S0 TX clock divider from hardware registers.
 * Must be called AFTER i2s_channel_init_std_mode(). */
void aes67_audio_read_clock_divider(aes67_audio_handle_t handle)
{
    (void)handle;
    base_div_n = HP_SYS_CLKRST.peri_clk_ctrl13.reg_i2s0_tx_div_n;
    base_div_x = HP_SYS_CLKRST.peri_clk_ctrl13.reg_i2s0_tx_div_x;
    base_div_y = HP_SYS_CLKRST.peri_clk_ctrl14.reg_i2s0_tx_div_y;
    base_div_z = HP_SYS_CLKRST.peri_clk_ctrl14.reg_i2s0_tx_div_z;
    base_div_yn1 = HP_SYS_CLKRST.peri_clk_ctrl14.reg_i2s0_tx_div_yn1;

    /* Reverse the x,y,z,yn1 -> numerator,denominator conversion */
    if (base_div_z > 0) {
        base_denominator = (base_div_x + 1) * base_div_z + base_div_y;
        base_numerator = base_div_yn1 ? (base_denominator - base_div_z) : base_div_z;
    }

    ESP_LOGI("aes67_audio", "I2S0 TX clk: N=%lu, x=%lu, y=%lu, z=%lu, yn1=%lu "
             "-> numer=%lu denom=%lu (divider=N+%lu/%lu)",
             (unsigned long)base_div_n, (unsigned long)base_div_x,
             (unsigned long)base_div_y, (unsigned long)base_div_z,
             (unsigned long)base_div_yn1,
             (unsigned long)base_numerator, (unsigned long)base_denominator,
             (unsigned long)base_numerator, (unsigned long)base_denominator);
}

/* Adjust the I2S TX MCLK divider at runtime to fine-tune sample rate.
 * This is used for adaptive clock recovery: speed up or slow down the
 * I2S clock to match the network source rate.
 * ppm_adj: parts-per-million adjustment. +100 = 0.01% faster. */
void aes67_audio_adjust_clock_ppm(aes67_audio_handle_t handle, int32_t ppm_adj)
{
    if (!handle || base_denominator == 0) return;

    /* Adjust numerator relative to the cached base values.
     * Increasing numerator = larger divider = slower clock.
     * ppm_adj > 0 means "source is faster, speed up I2S" = decrease numerator. */
    int64_t adj = (int64_t)ppm_adj * (int64_t)base_denominator / 1000000;
    int32_t new_numer = (int32_t)base_numerator - (int32_t)adj;
    if (new_numer < 1) new_numer = 1;
    if (new_numer > (int32_t)base_denominator - 1) new_numer = base_denominator - 1;

    /* Convert numerator/denominator back to x,y,z,yn1 format */
    uint32_t numer = (uint32_t)new_numer;
    uint32_t denom = base_denominator;
    uint32_t div_yn1 = numer * 2 > denom;
    uint32_t div_z = div_yn1 ? denom - numer : numer;
    uint32_t div_x = div_z ? (denom / div_z - 1) : 0;
    uint32_t div_y = div_z ? (denom % div_z) : 0;

    /* Write to I2S0 TX clock divider registers.
     * Workaround: set small div first, then target. */
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl13,
                                    reg_i2s0_tx_div_n, 2);
    HP_SYS_CLKRST.peri_clk_ctrl14.reg_i2s0_tx_div_yn1 = 0;
    HP_SYS_CLKRST.peri_clk_ctrl14.reg_i2s0_tx_div_y = 1;
    HP_SYS_CLKRST.peri_clk_ctrl14.reg_i2s0_tx_div_z = 0;
    HP_SYS_CLKRST.peri_clk_ctrl13.reg_i2s0_tx_div_x = 0;
    /* Set target */
    HP_SYS_CLKRST.peri_clk_ctrl14.reg_i2s0_tx_div_yn1 = div_yn1;
    HP_SYS_CLKRST.peri_clk_ctrl14.reg_i2s0_tx_div_z = div_z;
    HP_SYS_CLKRST.peri_clk_ctrl14.reg_i2s0_tx_div_y = div_y;
    HP_SYS_CLKRST.peri_clk_ctrl13.reg_i2s0_tx_div_x = div_x;
    HAL_FORCE_MODIFY_U32_REG_FIELD(HP_SYS_CLKRST.peri_clk_ctrl13,
                                    reg_i2s0_tx_div_n, base_div_n);
}

SemaphoreHandle_t aes67_audio_get_dma_sem(aes67_audio_handle_t handle)
{
    return handle ? handle->dma_sem : NULL;
}

i2s_chan_handle_t aes67_audio_get_tx_chan(aes67_audio_handle_t handle)
{
    return handle ? handle->tx_chan : NULL;
}
