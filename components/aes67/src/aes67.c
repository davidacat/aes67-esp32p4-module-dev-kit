/*
 * AES67/RAVENNA node lifecycle management.
 *
 * Coordinates initialization, startup and teardown of all AES67
 * subsystems: PTP, RTP engine, SAP discovery, audio I/O and the
 * session manager that ties them together.
 */

#include "aes67.h"
#include "aes67_config.h"
#include "aes67_ptp.h"
#include "aes67_rtp.h"
#include "aes67_sap.h"
#include "aes67_audio.h"
#include "aes67_session.h"
#include "aes67_hw_timer.h"

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "driver/i2s_std.h"

static const char *TAG = "aes67";

/* mDNS init/deinit live in aes67_mdns.c */
extern esp_err_t aes67_mdns_init(const char *node_name, uint16_t rtp_port);
extern void aes67_mdns_deinit(void);

/* Define the event base for the ESP event loop */
ESP_EVENT_DEFINE_BASE(AES67_EVENT);

/* Semaphore signaled by the hardware PTP timer ISR */
static SemaphoreHandle_t s_frame_sem = NULL;

/* Internal node structure holding all subsystem handles */
struct aes67_node {
    aes67_config_t              config;
    aes67_ptp_handle_t          ptp;
    aes67_rtp_engine_handle_t   rtp;
    aes67_sap_handle_t          sap;
    aes67_audio_handle_t        audio;
    aes67_session_handle_t      session;
    TaskHandle_t                tx_task;
    bool                        running;
};

/* Global node pointer for the ISR callback (single instance) */
static aes67_node_handle_t s_node = NULL;

/*
 * Hardware PTP timer ISR callback. Fires at exact PTP-clock-aligned
 * audio frame boundaries. Gives the semaphore to wake the TX task.
 */
static void IRAM_ATTR hw_frame_isr(void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_frame_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/*
 * Unified audio frame task (TIC). Driven by the hardware PTP timer ISR,
 * this task handles both TX and RX audio processing at PTP-aligned
 * frame boundaries:
 *
 * TX path: Read from source ring buffers -> convert -> send RTP packets
 * RX path: Read from sink jitter buffers -> write to I2S playback
 *
 * This ensures both directions are synchronized to the PTP clock with
 * hardware-precision timing.
 */
static void audio_frame_task(void *arg)
{
    aes67_node_handle_t node = (aes67_node_handle_t)arg;
    uint32_t frame_count = 0;

    /* Pre-allocate playback buffer for sink -> I2S path */
    uint32_t spp = (node->config.audio.sample_rate *
                     node->config.audio.packet_time_us) / 1000000;
    int32_t *playback_buf = heap_caps_malloc(
        spp * node->config.audio.channels * sizeof(int32_t),
        MALLOC_CAP_INTERNAL);

    ESP_LOGI(TAG, "Audio frame task started (hw-timed TX+RX)");

    while (node->running) {
        /* Wait for the hardware PTP frame timer ISR */
        if (xSemaphoreTake(s_frame_sem, pdMS_TO_TICKS(10)) != pdTRUE) {
            continue;
        }

        frame_count++;

        /* Get current PTP time for RTP timestamp generation */
        uint64_t ptp_time_ns;
        aes67_ptp_get_time_ns(node->ptp, &ptp_time_ns);
        uint32_t rtp_ts = aes67_ptp_time_to_rtp_ts(
            ptp_time_ns, node->config.audio.sample_rate);

        /* TX: Process all active source streams - read from ring buffers,
         * convert to network format, send RTP packets */
        aes67_rtp_engine_process_tx(node->rtp, rtp_ts);

        /* RX playback is handled by a dedicated playback task.
         * The TIC task only handles TX (source streams). */
    }

    ESP_LOGI(TAG, "Audio frame task stopped");
    vTaskDelete(NULL);
}

/*
 * Dedicated playback task. Runs in a tight loop reading from the
 * first active sink's jitter buffer and writing to I2S.
 * i2s_channel_write blocks until DMA has room, which naturally
 * paces output at 48kHz. This task is completely decoupled from
 * both the RTP RX task and the TIC task.
 */
static void playback_task(void *arg)
{
    aes67_node_handle_t node = (aes67_node_handle_t)arg;

    /* Compute stream params from config */
    aes67_stream_params_t params;
    aes67_stream_params_compute(&params,
                                node->config.audio.sample_rate,
                                node->config.audio.channels,
                                node->config.audio.word_length,
                                node->config.audio.packet_time_us);

    const uint32_t spp = params.samples_per_packet;
    const uint32_t max_packets = 3;  /* Read up to 3 packets per cycle */
    const uint8_t ch = params.channels;
    const uint32_t pkt_native_bytes = params.packet_native_size;

    int32_t *buf = heap_caps_malloc(pkt_native_bytes, MALLOC_CAP_INTERNAL);
    if (!buf) {
        ESP_LOGE(TAG, "Playback task: alloc failed");
        vTaskDelete(NULL);
        return;
    }

    /* Allocate playback double-buffers (sized to actual stream params) */
    int32_t *pcm_buf = heap_caps_malloc(pkt_native_bytes, MALLOC_CAP_INTERNAL);
    int32_t *prev_buf = heap_caps_malloc(pkt_native_bytes, MALLOC_CAP_INTERNAL);
    if (!pcm_buf || !prev_buf) {
        ESP_LOGE(TAG, "Playback task: double-buffer alloc failed");
        heap_caps_free(buf);
        heap_caps_free(pcm_buf);
        heap_caps_free(prev_buf);
        vTaskDelete(NULL);
        return;
    }
    bool have_prev = false;

    extern esp_err_t aes67_audio_direct_write(
        aes67_audio_handle_t handle,
        const int32_t *samples, uint32_t frame_count);
    extern uint32_t aes67_rtp_sink_available(aes67_rtp_stream_handle_t stream);

    ESP_LOGI(TAG, "Playback task started (spp=%lu, ch=%u, pkt=%lu bytes)",
             (unsigned long)spp, ch, (unsigned long)pkt_native_bytes);

    aes67_rtp_stream_handle_t cached_sink = NULL;
    bool prefilled = false;
    uint32_t total_frames = 0;
    uint32_t write_count = 0;
    int64_t start_us = 0;

    while (node->running) {
        if (!cached_sink) {
            prefilled = false;
            int sink_count = aes67_session_get_sink_count(node->session);
            for (int s = 0; s < sink_count && s < CONFIG_AES67_MAX_SINKS; s++) {
                aes67_sink_t sink;
                if (aes67_session_get_sink(node->session, s, &sink) == ESP_OK &&
                    sink.enabled && sink.rtp_stream) {
                    cached_sink = sink.rtp_stream;
                    ESP_LOGI(TAG, "Playback: locked to sink '%s' "
                             "(ch=%u, wl=%u, rate=%lu, ptime=%lu us)",
                             sink.name, sink.rtp_config.channels,
                             sink.rtp_config.word_length,
                             (unsigned long)sink.rtp_config.sample_rate,
                             (unsigned long)sink.rtp_config.packet_time_us);
                    break;
                }
            }
            if (!cached_sink) {
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
        }

        if (!prefilled) {
            prefilled = true;
            start_us = esp_timer_get_time();
            ESP_LOGI(TAG, "Playback: stream buffer tight-loop mode");
        }

        /* Try external hook stream buffer first, fall back to RTP engine's */
        extern StreamBufferHandle_t s_hook_sbuf;
        extern StreamBufferHandle_t aes67_rtp_engine_get_stream_buf(void *h);
        StreamBufferHandle_t sbuf = s_hook_sbuf ? s_hook_sbuf :
                                     aes67_rtp_engine_get_stream_buf(node->rtp);
        if (!sbuf) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* DMA-paced playback with auto_clear=false:
         * 1. Write previous data to I2S (BLOCKS until DMA frees descriptor)
         * 2. During the block, raw callback fills stream buffer
         * 3. Read new data from stream buffer (should be ready instantly)
         * 4. Loop to step 1 with the new data
         *
         * The i2s_channel_write blocking IS the sample-rate pacing mechanism.
         * With auto_clear=false, if we don't write, DMA replays old audio. */
        const uint32_t pkt_bytes = pkt_native_bytes;
        uint32_t frames = spp;

        extern i2s_chan_handle_t aes67_audio_get_tx_chan(void *h);
        i2s_chan_handle_t tx = aes67_audio_get_tx_chan(node->audio);

        if (have_prev && tx) {
            size_t written = 0;
            i2s_channel_write(tx, prev_buf, pkt_bytes,
                              &written, portMAX_DELAY);
        }

        size_t avail = xStreamBufferBytesAvailable(sbuf);
        if (avail >= pkt_bytes) {
            xStreamBufferReceive(sbuf, pcm_buf, pkt_bytes, 0);
            memcpy(prev_buf, pcm_buf, pkt_bytes);
            have_prev = true;
        } else if (!have_prev) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        /* If no new data but have_prev: prev_buf stays unchanged,
         * next write will replay it. DMA also replays old descriptors
         * with auto_clear=false, so the output is smooth. */

        total_frames += frames;
        write_count++;

        /* Show RECENT fps (last 1000 writes) instead of cumulative average */
        if ((write_count % 1000) == 0) {
            static int64_t last_stat_us = 0;
            static uint32_t last_stat_frames = 0;
            int64_t now = esp_timer_get_time();
            if (last_stat_us > 0) {
                int64_t delta_us = now - last_stat_us;
                uint32_t delta_frames = total_frames - last_stat_frames;
                if (delta_us > 0) {
                    ESP_LOGI(TAG, "PB: %lu fps, sb=%u",
                             (unsigned long)(delta_frames * 1000000ULL / delta_us),
                             (unsigned)xStreamBufferBytesAvailable(sbuf));
                }
            }
            last_stat_us = now;
            last_stat_frames = total_frames;
        }
    }

    heap_caps_free(buf);
    heap_caps_free(pcm_buf);
    heap_caps_free(prev_buf);
    ESP_LOGI(TAG, "Playback task stopped");
    vTaskDelete(NULL);
}

/*
 * Audio frame callback from I2S DMA (unused - kept for API compat).
 */
static void audio_frame_callback(uint32_t frame_count, void *user_data)
{
}

esp_err_t aes67_node_init(const aes67_config_t *config, aes67_node_handle_t *handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    struct aes67_node *node = calloc(1, sizeof(struct aes67_node));
    if (!node) {
        ESP_LOGE(TAG, "failed to allocate node context");
        return ESP_ERR_NO_MEM;
    }

    /* Apply caller config or fall back to Kconfig defaults */
    if (config) {
        memcpy(&node->config, config, sizeof(aes67_config_t));
    } else {
        aes67_config_t defaults = AES67_CONFIG_DEFAULT();
        memcpy(&node->config, &defaults, sizeof(aes67_config_t));
    }

    esp_err_t ret;

    /* 1. PTP clock subsystem */
    ret = aes67_ptp_init(node->config.net.eth_handle, &node->config.ptp, &node->ptp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PTP init failed: %s", esp_err_to_name(ret));
        goto fail_ptp;
    }

    /* 2. RTP packet engine */
    ret = aes67_rtp_engine_init(&node->config.net, node->config.use_psram, &node->rtp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTP engine init failed: %s", esp_err_to_name(ret));
        goto fail_rtp;
    }

    /* 3. SAP discovery (optional) */
    if (node->config.sap_enabled) {
        ret = aes67_sap_init(&node->sap);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SAP init failed: %s", esp_err_to_name(ret));
            goto fail_sap;
        }
    }

    /* 4. Audio I/O driver (I2S + DMA) */
    ret = aes67_audio_init(&node->config.audio, &node->config.i2s_pins,
                           node->config.use_psram, node->config.ring_buffer_ms,
                           &node->audio);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "audio init failed: %s", esp_err_to_name(ret));
        goto fail_audio;
    }

    /* 5. Session manager -- connects all subsystems */
    ret = aes67_session_init(node->rtp, node->ptp, node->sap,
                             node->audio, &node->config, &node->session);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "session manager init failed: %s", esp_err_to_name(ret));
        goto fail_session;
    }

    /* 6. Register the audio frame callback (for I2S DMA capture handling) */
    ret = aes67_audio_register_frame_cb(node->audio, audio_frame_callback, node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "audio frame callback registration failed: %s",
                 esp_err_to_name(ret));
        goto fail_cb;
    }

    /* 7. Initialize hardware PTP frame timer for RTP TX.
     * frame_period = samples_per_packet / sample_rate in nanoseconds */
    uint32_t samples_per_pkt = (node->config.audio.sample_rate *
                                 node->config.audio.packet_time_us) / 1000000;
    uint32_t frame_ns = (uint32_t)((uint64_t)samples_per_pkt * 1000000000ULL /
                                    node->config.audio.sample_rate);

    /* Create the frame semaphore for ISR -> task signaling */
    s_frame_sem = xSemaphoreCreateBinary();
    if (!s_frame_sem) {
        ESP_LOGE(TAG, "failed to create frame semaphore");
        goto fail_cb;
    }

    ret = aes67_hw_timer_init(frame_ns, hw_frame_isr, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "hw PTP timer init failed: %s (falling back to I2S timing)",
                 esp_err_to_name(ret));
        /* Non-fatal: I2S DMA callback still works as fallback */
    }

    s_node = node;
    node->running = false;
    *handle = node;

    ESP_LOGI(TAG, "node \"%s\" initialized (v%d.%d.%d)",
             node->config.node_name,
             AES67_VERSION_MAJOR, AES67_VERSION_MINOR, AES67_VERSION_PATCH);

    return ESP_OK;

fail_cb:
    aes67_session_destroy(node->session);
fail_session:
    aes67_audio_destroy(node->audio);
fail_audio:
    if (node->config.sap_enabled && node->sap) {
        aes67_sap_destroy(node->sap);
    }
fail_sap:
    aes67_rtp_engine_destroy(node->rtp);
fail_rtp:
    aes67_ptp_destroy(node->ptp);
fail_ptp:
    free(node);
    return ret;
}

esp_err_t aes67_node_start(aes67_node_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->running) {
        ESP_LOGW(TAG, "node already running");
        return ESP_OK;
    }

    esp_err_t ret;

    /* Start subsystems in dependency order */
    ret = aes67_ptp_start(handle->ptp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PTP start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = aes67_rtp_engine_start(handle->rtp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTP engine start failed: %s", esp_err_to_name(ret));
        aes67_ptp_stop(handle->ptp);
        return ret;
    }

    if (handle->config.sap_enabled && handle->sap) {
        ret = aes67_sap_start(handle->sap);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SAP start failed: %s", esp_err_to_name(ret));
            aes67_rtp_engine_stop(handle->rtp);
            aes67_ptp_stop(handle->ptp);
            return ret;
        }
    }

    ret = aes67_audio_start(handle->audio);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "audio start failed: %s", esp_err_to_name(ret));
        if (handle->config.sap_enabled && handle->sap) {
            aes67_sap_stop(handle->sap);
        }
        aes67_rtp_engine_stop(handle->rtp);
        aes67_ptp_stop(handle->ptp);
        return ret;
    }

    /* mDNS advertisement (optional) */
    if (handle->config.mdns_enabled) {
        ret = aes67_mdns_init(handle->config.node_name, handle->config.net.rtp_port);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "mDNS init failed: %s (continuing without mDNS)",
                     esp_err_to_name(ret));
            /* Non-fatal: the node still works without mDNS */
        }
    }

    handle->running = true;

    /* Enable direct RTP RX -> staging ring path.
     * RTP RX task converts and writes to staging inline,
     * DMA ISR drains staging to I2S. No jitter buffer or playback task. */
    extern void aes67_rtp_engine_set_playback(aes67_rtp_engine_handle_t h,
                                               void *audio);
    aes67_rtp_engine_set_playback(handle->rtp, handle->audio);
    ESP_LOGI(TAG, "Direct RTP->I2S path enabled (zero-buffer)");

    /* Start dedicated playback task AFTER running=true */
    {
        static TaskHandle_t pb_task = NULL;
        /* Playback on core 1 at priority 20 (below lwIP 22, above RX 17).
         * lwIP processes packets first, then playback drains the jitter
         * buffer, then RX picks up from the socket. */
        /* Playback on core 1 at priority 21 (above RTP RX 17, below lwIP 22).
         * Was on core 0 where TIC task (prio 21) preempted it every 1ms. */
        xTaskCreatePinnedToCore(playback_task, "aes67_pb", 8192, handle,
                                21, &pb_task, 1);

        /* Register playback task for notification from RTP RX */
        extern void aes67_rtp_engine_set_notify_task(
            aes67_rtp_engine_handle_t h, TaskHandle_t task);
        aes67_rtp_engine_set_notify_task(handle->rtp, pb_task);
    }

    /* Start the hardware PTP frame timer and audio frame task.
     * This task handles both TX and RX at PTP-aligned boundaries. */
    ret = aes67_hw_timer_start();
    if (ret == ESP_OK) {
        BaseType_t xret = xTaskCreatePinnedToCore(
            audio_frame_task, "aes67_tic", 8192, handle,
            21,     /* High priority, just below audio I/O (22) */
            &handle->tx_task,
            0       /* Pin to core 0 (PTP/network core) */
        );
        if (xret != pdPASS) {
            ESP_LOGW(TAG, "failed to create audio frame task");
            aes67_hw_timer_stop();
        } else {
            ESP_LOGI(TAG, "Audio frame processing driven by hardware PTP timer");
        }
    } else {
        ESP_LOGW(TAG, "hw timer start failed, using I2S DMA callback");
    }

    ESP_LOGI(TAG, "node started");

    return ESP_OK;
}

esp_err_t aes67_node_stop(aes67_node_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!handle->running) {
        ESP_LOGW(TAG, "node not running");
        return ESP_OK;
    }

    /* Tear down in reverse order */
    if (handle->config.mdns_enabled) {
        aes67_mdns_deinit();
    }

    aes67_audio_stop(handle->audio);

    if (handle->config.sap_enabled && handle->sap) {
        aes67_sap_stop(handle->sap);
    }

    aes67_rtp_engine_stop(handle->rtp);
    aes67_ptp_stop(handle->ptp);

    handle->running = false;
    ESP_LOGI(TAG, "node stopped");

    return ESP_OK;
}

esp_err_t aes67_node_destroy(aes67_node_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Stop first if still running */
    if (handle->running) {
        aes67_node_stop(handle);
    }

    /* Destroy subsystems in reverse init order */
    aes67_session_destroy(handle->session);
    aes67_audio_destroy(handle->audio);

    if (handle->config.sap_enabled && handle->sap) {
        aes67_sap_destroy(handle->sap);
    }

    aes67_rtp_engine_destroy(handle->rtp);
    aes67_ptp_destroy(handle->ptp);

    ESP_LOGI(TAG, "node \"%s\" destroyed", handle->config.node_name);
    free(handle);

    return ESP_OK;
}

esp_err_t aes67_node_get_config(aes67_node_handle_t handle, aes67_config_t *config)
{
    if (!handle || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    memcpy(config, &handle->config, sizeof(aes67_config_t));
    return ESP_OK;
}

esp_err_t aes67_node_get_session(aes67_node_handle_t handle,
                                  aes67_session_handle_t *session)
{
    if (!handle || !session) {
        return ESP_ERR_INVALID_ARG;
    }

    *session = handle->session;
    return ESP_OK;
}

esp_err_t aes67_node_get_audio(aes67_node_handle_t handle,
                               aes67_audio_handle_t *audio)
{
    if (!handle || !audio) {
        return ESP_ERR_INVALID_ARG;
    }
    *audio = handle->audio;
    return ESP_OK;
}

esp_err_t aes67_node_get_sap(aes67_node_handle_t handle,
                              aes67_sap_handle_t *sap)
{
    if (!handle || !sap) {
        return ESP_ERR_INVALID_ARG;
    }

    *sap = handle->sap;
    return ESP_OK;
}
