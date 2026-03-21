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

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_event.h"

static const char *TAG = "aes67";

/* mDNS init/deinit live in aes67_mdns.c */
extern esp_err_t aes67_mdns_init(const char *node_name, uint16_t rtp_port);
extern void aes67_mdns_deinit(void);

/* Define the event base for the ESP event loop */
ESP_EVENT_DEFINE_BASE(AES67_EVENT);

/* Internal node structure holding all subsystem handles */
struct aes67_node {
    aes67_config_t              config;
    aes67_ptp_handle_t          ptp;
    aes67_rtp_engine_handle_t   rtp;
    aes67_sap_handle_t          sap;
    aes67_audio_handle_t        audio;
    aes67_session_handle_t      session;
    bool                        running;
};

/*
 * Audio frame callback (TIC). Called from the audio driver's DMA ISR context
 * once per frame period. Derives an RTP timestamp from the current PTP time
 * and triggers packet transmission for all active source streams.
 */
static void audio_frame_callback(uint32_t frame_count, void *user_data)
{
    aes67_node_handle_t node = (aes67_node_handle_t)user_data;

    uint64_t ptp_time_ns;
    aes67_ptp_get_time_ns(node->ptp, &ptp_time_ns);

    uint32_t rtp_ts = aes67_ptp_time_to_rtp_ts(ptp_time_ns,
                                                 node->config.audio.sample_rate);
    aes67_rtp_engine_process_tx(node->rtp, rtp_ts);
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

    /* 6. Register the audio frame callback that drives RTP TX timing */
    ret = aes67_audio_register_frame_cb(node->audio, audio_frame_callback, node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "audio frame callback registration failed: %s",
                 esp_err_to_name(ret));
        goto fail_cb;
    }

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
