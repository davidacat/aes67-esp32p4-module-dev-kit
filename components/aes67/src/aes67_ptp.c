/*
 * PTP (IEEE 1588) clock synchronization implementation.
 *
 * Wraps the ptpd daemon and ESP32-P4 hardware timestamping to provide
 * a synchronized PTP clock for AES67 RTP timestamp generation.
 * Monitors lock state and notifies upper layers on transitions.
 */

#include "aes67_ptp.h"

#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_eth.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "aes67_ptp";

/* Thresholds for lock state machine */
#define PTP_LOCKING_THRESHOLD_NS    (10 * 1000 * 1000)   /* 10 ms  */
#define PTP_LOCKED_THRESHOLD_NS     (1000)                /* 1 us   */
#define PTP_LOCKED_STABLE_COUNT     4                     /* iterations at < 1us before locked */
#define PTP_MONITOR_INTERVAL_MS     500
#define PTP_MONITOR_STACK_SIZE      4096
#define PTP_MONITOR_PRIORITY        5
#define PTP_INTERFACE_NAME          "ETH_0"

/* PTP daemon and clock APIs from local components */
#include "ptpd.h"
#include "esp_eth_time.h"
#include "esp_vfs_l2tap.h"

/* -------------------------------------------------------------------
 * Internal context
 * ------------------------------------------------------------------- */
struct aes67_ptp_ctx {
    esp_eth_handle_t eth_handle;
    int ptpd_pid;
    aes67_ptp_config_t config;
    aes67_ptp_lock_state_t lock_state;
    aes67_ptp_lock_cb_t lock_cb;
    void *lock_cb_user_data;
    TaskHandle_t monitor_task;
    bool running;

    /* Grandmaster identity from last status poll */
    uint8_t grandmaster_id[8];

    /* Own MAC address, used for generating grandmaster identity when acting as GM */
    uint8_t own_mac[6];
    bool is_grandmaster;            /* True when this node is acting as PTP GM */

    /* Rolling count of consecutive samples under the locked threshold */
    int stable_count;

    /* Cached status values */
    int32_t offset_ns;
    int32_t path_delay_ns;
};

/* -------------------------------------------------------------------
 * Lock state machine
 * ------------------------------------------------------------------- */

/*
 * Evaluate offset and advance the lock state machine.
 * Returns true if the state changed.
 */
static bool ptp_update_lock_state(struct aes67_ptp_ctx *ctx, int64_t offset_ns)
{
    int64_t abs_offset = (offset_ns < 0) ? -offset_ns : offset_ns;
    aes67_ptp_lock_state_t prev = ctx->lock_state;

    switch (ctx->lock_state) {
    case AES67_PTP_UNLOCKED:
        if (abs_offset < PTP_LOCKING_THRESHOLD_NS) {
            ctx->lock_state = AES67_PTP_LOCKING;
            ctx->stable_count = 0;
            ESP_LOGI(TAG, "PTP state: UNLOCKED -> LOCKING (offset %lld ns)", (long long)offset_ns);
        }
        break;

    case AES67_PTP_LOCKING:
        if (abs_offset >= PTP_LOCKING_THRESHOLD_NS) {
            /* Lost convergence, fall back */
            ctx->lock_state = AES67_PTP_UNLOCKED;
            ctx->stable_count = 0;
            ESP_LOGW(TAG, "PTP state: LOCKING -> UNLOCKED (offset %lld ns)", (long long)offset_ns);
        } else if (abs_offset < PTP_LOCKED_THRESHOLD_NS) {
            ctx->stable_count++;
            if (ctx->stable_count >= PTP_LOCKED_STABLE_COUNT) {
                ctx->lock_state = AES67_PTP_LOCKED;
                ESP_LOGI(TAG, "PTP state: LOCKING -> LOCKED (offset %lld ns, %d stable polls)",
                         (long long)offset_ns, ctx->stable_count);
            }
        } else {
            /* Still converging but not yet sub-microsecond */
            ctx->stable_count = 0;
        }
        break;

    case AES67_PTP_LOCKED:
        if (abs_offset >= PTP_LOCKING_THRESHOLD_NS) {
            ctx->lock_state = AES67_PTP_UNLOCKED;
            ctx->stable_count = 0;
            ESP_LOGW(TAG, "PTP state: LOCKED -> UNLOCKED (offset %lld ns)", (long long)offset_ns);
        } else if (abs_offset >= PTP_LOCKED_THRESHOLD_NS) {
            ctx->lock_state = AES67_PTP_LOCKING;
            ctx->stable_count = 0;
            ESP_LOGW(TAG, "PTP state: LOCKED -> LOCKING (offset %lld ns)", (long long)offset_ns);
        }
        break;
    }

    return (ctx->lock_state != prev);
}

/* -------------------------------------------------------------------
 * Monitor task
 * ------------------------------------------------------------------- */

static void ptp_monitor_task(void *arg)
{
    struct aes67_ptp_ctx *ctx = (struct aes67_ptp_ctx *)arg;
    struct ptpd_status_s status;

    ESP_LOGI(TAG, "PTP monitor task started");

    while (ctx->running) {
        vTaskDelay(pdMS_TO_TICKS(PTP_MONITOR_INTERVAL_MS));

        if (!ctx->running) {
            break;
        }

        memset(&status, 0, sizeof(status));
        if (ctx->ptpd_pid < 0) {
            continue;
        }
        int ret = ptpd_status(ctx->ptpd_pid, &status);
        if (ret != 0) {
            ESP_LOGD(TAG, "ptpd_status returned %d, daemon may still be initializing", ret);
            continue;
        }

        /* Cache latest offset and path delay */
        ctx->offset_ns = (int32_t)status.last_delta_ns;
        ctx->path_delay_ns = (int32_t)status.path_delay_ns;

        /* Extract grandmaster identity from status struct */
        memcpy(ctx->grandmaster_id, status.clock_source_info.gm_id, 8);

        /* Detect if we are the grandmaster. When acting as GM, the ptpd daemon
         * will report no valid clock source, or the GM ID will match our own
         * EUI-64 (MAC with FF:FE inserted in the middle). */
        bool is_self_gm = !status.clock_source_valid;
        if (!is_self_gm) {
            is_self_gm = (
            ctx->grandmaster_id[0] == ctx->own_mac[0] &&
            ctx->grandmaster_id[1] == ctx->own_mac[1] &&
            ctx->grandmaster_id[2] == ctx->own_mac[2] &&
            ctx->grandmaster_id[3] == 0xFF &&
            ctx->grandmaster_id[4] == 0xFE &&
            ctx->grandmaster_id[5] == ctx->own_mac[3] &&
            ctx->grandmaster_id[6] == ctx->own_mac[4] &&
            ctx->grandmaster_id[7] == ctx->own_mac[5]);
        }

        if (is_self_gm) {

            if (!ctx->is_grandmaster) {
                ctx->is_grandmaster = true;
                ESP_LOGI(TAG, "This node is PTP grandmaster "
                         "(GM ID: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X)",
                         ctx->own_mac[0], ctx->own_mac[1], ctx->own_mac[2],
                         0xFF, 0xFE,
                         ctx->own_mac[3], ctx->own_mac[4], ctx->own_mac[5]);
            }

            /* When acting as grandmaster, we are always "locked" to our own clock */
            if (ctx->lock_state != AES67_PTP_LOCKED) {
                aes67_ptp_lock_state_t prev = ctx->lock_state;
                ctx->lock_state = AES67_PTP_LOCKED;
                ctx->offset_ns = 0;
                ctx->path_delay_ns = 0;
                if (prev != AES67_PTP_LOCKED && ctx->lock_cb) {
                    ctx->lock_cb(ctx->lock_state, ctx->lock_cb_user_data);
                }
            }
        } else {
            if (ctx->is_grandmaster) {
                ctx->is_grandmaster = false;
                ESP_LOGI(TAG, "External grandmaster detected, switching to slave mode "
                         "(GM ID: %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X)",
                         ctx->grandmaster_id[0], ctx->grandmaster_id[1],
                         ctx->grandmaster_id[2], ctx->grandmaster_id[3],
                         ctx->grandmaster_id[4], ctx->grandmaster_id[5],
                         ctx->grandmaster_id[6], ctx->grandmaster_id[7]);
            }

            /* Run lock state machine for slave mode */
            bool changed = ptp_update_lock_state(ctx, status.last_delta_ns);
            if (changed && ctx->lock_cb) {
                ctx->lock_cb(ctx->lock_state, ctx->lock_cb_user_data);
            }
        }
    }

    ESP_LOGI(TAG, "PTP monitor task exiting");
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------- */

esp_err_t aes67_ptp_init(esp_eth_handle_t eth_handle, const aes67_ptp_config_t *config,
                         aes67_ptp_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(eth_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "eth_handle is NULL");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is NULL");
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");

    struct aes67_ptp_ctx *ctx = calloc(1, sizeof(struct aes67_ptp_ctx));
    ESP_RETURN_ON_FALSE(ctx != NULL, ESP_ERR_NO_MEM, TAG, "failed to allocate PTP context");

    ctx->eth_handle = eth_handle;
    ctx->config = *config;
    ctx->lock_state = AES67_PTP_UNLOCKED;
    ctx->ptpd_pid = -1;
    ctx->running = false;
    ctx->is_grandmaster = false;

    /* Read the MAC address for grandmaster identity detection.
     * The EUI-64 GM ID is derived by inserting FF:FE into the MAC. */
    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, ctx->own_mac);

    /* Register the L2 TAP VFS interface. The ptpd component opens
     * /dev/net/tap for Layer 2 PTP packet transport with hardware
     * timestamping. This must happen before ptpd_start(). */
    esp_err_t ret = esp_vfs_l2tap_intf_register(NULL);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means already registered, which is fine */
        ESP_LOGE(TAG, "esp_vfs_l2tap_intf_register failed: %s", esp_err_to_name(ret));
        free(ctx);
        return ret;
    }

    /* Initialize the hardware PTP clock tied to the EMAC */
    esp_eth_clock_cfg_t clk_cfg = {
        .eth_hndl = eth_handle,
    };
    ret = esp_eth_clock_init(CLOCK_PTP_SYSTEM, &clk_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_clock_init failed: %s", esp_err_to_name(ret));
        free(ctx);
        return ret;
    }

    ESP_LOGI(TAG, "PTP subsystem initialized (domain %u, dscp %u)", config->domain, config->dscp);
    *handle = ctx;
    return ESP_OK;
}

esp_err_t aes67_ptp_start(aes67_ptp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    ESP_RETURN_ON_FALSE(!handle->running, ESP_ERR_INVALID_STATE, TAG, "PTP already running");

    /* Start the ptpd daemon. The interface name passed here must match
     * what the ESP netif was created with. For default ethernet config
     * this is typically "ETH_DEF". */
    int pid = ptpd_start(PTP_INTERFACE_NAME);
    if (pid < 0) {
        ESP_LOGE(TAG, "ptpd_start failed (returned %d)", pid);
        return ESP_FAIL;
    }

    handle->ptpd_pid = pid;
    handle->running = true;
    handle->lock_state = AES67_PTP_UNLOCKED;
    handle->stable_count = 0;

    ESP_LOGI(TAG, "ptpd started (pid %d)", pid);

    /* Spawn monitor task to track lock state */
    BaseType_t xret = xTaskCreate(
        ptp_monitor_task,
        "ptp_monitor",
        PTP_MONITOR_STACK_SIZE,
        handle,
        PTP_MONITOR_PRIORITY,
        &handle->monitor_task
    );
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "failed to create PTP monitor task");
        ptpd_stop(handle->ptpd_pid);
        handle->running = false;
        handle->ptpd_pid = -1;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t aes67_ptp_stop(aes67_ptp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    ESP_RETURN_ON_FALSE(handle->running, ESP_ERR_INVALID_STATE, TAG, "PTP not running");

    /* Signal the monitor task to stop, then wait for it to exit */
    handle->running = false;

    /* Give the monitor task time to notice the flag and exit */
    vTaskDelay(pdMS_TO_TICKS(PTP_MONITOR_INTERVAL_MS + 100));
    handle->monitor_task = NULL;

    /* Stop the ptpd daemon */
    int ret = ptpd_stop(handle->ptpd_pid);
    if (ret != 0) {
        ESP_LOGW(TAG, "ptpd_stop returned %d", ret);
    }
    handle->ptpd_pid = -1;

    /* Reset state */
    aes67_ptp_lock_state_t prev_state = handle->lock_state;
    handle->lock_state = AES67_PTP_UNLOCKED;
    handle->stable_count = 0;
    handle->offset_ns = 0;
    handle->path_delay_ns = 0;

    /* Notify if state changed */
    if (prev_state != AES67_PTP_UNLOCKED && handle->lock_cb) {
        handle->lock_cb(AES67_PTP_UNLOCKED, handle->lock_cb_user_data);
    }

    ESP_LOGI(TAG, "PTP stopped");
    return ESP_OK;
}

esp_err_t aes67_ptp_destroy(aes67_ptp_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");

    /* Stop first if still running */
    if (handle->running) {
        esp_err_t ret = aes67_ptp_stop(handle);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "aes67_ptp_stop returned %s during destroy", esp_err_to_name(ret));
        }
    }

    free(handle);
    ESP_LOGI(TAG, "PTP context destroyed");
    return ESP_OK;
}

esp_err_t aes67_ptp_get_status(aes67_ptp_handle_t handle, aes67_ptp_status_t *status)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG, "status is NULL");

    status->lock_state = handle->lock_state;
    status->domain = handle->config.domain;
    status->offset_ns = handle->offset_ns;
    status->path_delay_ns = handle->path_delay_ns;
    status->jitter_ns = 0;

    /* When acting as grandmaster, report our own EUI-64 as the GM ID */
    if (handle->is_grandmaster) {
        status->grandmaster_id[0] = handle->own_mac[0];
        status->grandmaster_id[1] = handle->own_mac[1];
        status->grandmaster_id[2] = handle->own_mac[2];
        status->grandmaster_id[3] = 0xFF;
        status->grandmaster_id[4] = 0xFE;
        status->grandmaster_id[5] = handle->own_mac[3];
        status->grandmaster_id[6] = handle->own_mac[4];
        status->grandmaster_id[7] = handle->own_mac[5];
    } else {
        memcpy(status->grandmaster_id, handle->grandmaster_id, 8);
    }

    return ESP_OK;
}

esp_err_t aes67_ptp_get_time_ns(aes67_ptp_handle_t handle, uint64_t *time_ns)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    ESP_RETURN_ON_FALSE(time_ns != NULL, ESP_ERR_INVALID_ARG, TAG, "time_ns is NULL");

    struct timespec ts;
    int ret = esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &ts);
    if (ret != 0) {
        ESP_LOGE(TAG, "esp_eth_clock_gettime failed (%d)", ret);
        return ESP_FAIL;
    }

    *time_ns = (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
    return ESP_OK;
}

esp_err_t aes67_ptp_get_time(aes67_ptp_handle_t handle, uint32_t *sec, uint32_t *nsec)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    ESP_RETURN_ON_FALSE(sec != NULL && nsec != NULL, ESP_ERR_INVALID_ARG, TAG, "output is NULL");

    struct timespec ts;
    int ret = esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &ts);
    if (ret != 0) {
        ESP_LOGE(TAG, "esp_eth_clock_gettime failed (%d)", ret);
        return ESP_FAIL;
    }

    *sec = (uint32_t)ts.tv_sec;
    *nsec = (uint32_t)ts.tv_nsec;
    return ESP_OK;
}

uint32_t aes67_ptp_time_to_rtp_ts(uint64_t ptp_time_ns, uint32_t sample_rate)
{
    /*
     * RTP timestamp = (ptp_time_ns * sample_rate) / 1_000_000_000
     *
     * To avoid 128-bit overflow we split the computation:
     *   seconds part: (ptp_time_ns / 1e9) * sample_rate
     *   sub-second part: (ptp_time_ns % 1e9) * sample_rate / 1e9
     *
     * The result is taken modulo 2^32 (implicit uint32_t wrap).
     */
    uint64_t sec = ptp_time_ns / 1000000000ULL;
    uint64_t nsec = ptp_time_ns % 1000000000ULL;

    uint64_t ts = (sec * sample_rate) + (nsec * sample_rate / 1000000000ULL);
    return (uint32_t)ts;
}

esp_err_t aes67_ptp_register_lock_cb(aes67_ptp_handle_t handle,
                                     aes67_ptp_lock_cb_t cb, void *user_data)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");

    handle->lock_cb = cb;
    handle->lock_cb_user_data = user_data;
    return ESP_OK;
}

esp_err_t aes67_ptp_get_grandmaster_str(aes67_ptp_handle_t handle, char *buf, size_t len)
{
    ESP_RETURN_ON_FALSE(handle != NULL, ESP_ERR_INVALID_ARG, TAG, "handle is NULL");
    ESP_RETURN_ON_FALSE(buf != NULL, ESP_ERR_INVALID_ARG, TAG, "buf is NULL");
    ESP_RETURN_ON_FALSE(len >= 24, ESP_ERR_INVALID_ARG, TAG,
                        "buf too small (need 24 bytes for \"XX-XX-XX-XX-XX-XX-XX-XX\")");

    const uint8_t *gm = handle->grandmaster_id;
    snprintf(buf, len, "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
             gm[0], gm[1], gm[2], gm[3], gm[4], gm[5], gm[6], gm[7]);

    return ESP_OK;
}
