/*
 * Hardware PTP-synchronized audio frame timer.
 *
 * Uses the ESP32-P4 EMAC's target time interrupt to fire a callback
 * at precise PTP-clock-aligned intervals. The target time mechanism
 * programs an absolute PTP timestamp; when the EMAC PTP counter
 * reaches that value, a hardware ISR fires. We use this to create
 * a periodic frame timer synchronized to the PTP clock.
 *
 * This eliminates software timing jitter from vTaskDelay or software
 * timers, giving nanosecond-precision audio frame boundaries that
 * track the PTP master clock.
 */

#include "aes67_hw_timer.h"
#include "esp_eth_time.h"
#include "esp_log.h"
#include "esp_attr.h"

#include <string.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "aes67_hw_timer";

static struct {
    uint32_t frame_period_ns;
    aes67_hw_timer_cb_t cb;
    void *user_data;
    struct timespec next_time;
    bool running;
} s_timer;

/* Add nanoseconds to a timespec, handling overflow */
static inline void timespec_add_ns(struct timespec *ts, uint32_t ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_nsec -= 1000000000L;
        ts->tv_sec++;
    }
}

/* ISR callback from EMAC when PTP clock reaches target time.
 * Runs in ISR context - must be fast and in IRAM. */
IRAM_ATTR static bool hw_timer_isr(esp_eth_mediator_t *eth, void *user_args)
{
    if (!s_timer.running) {
        return false;
    }

    /* Fire the user callback */
    if (s_timer.cb) {
        s_timer.cb(s_timer.user_data);
    }

    /* Schedule the next target time */
    timespec_add_ns(&s_timer.next_time, s_timer.frame_period_ns);

    /* Verify the next time is still in the future. If we're behind
     * (e.g., after a PTP clock jump), re-anchor to now + one period
     * instead of looping to catch up (which would hang the ISR). */
    struct timespec now;
    esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &now);

    int64_t diff_ns = ((int64_t)s_timer.next_time.tv_sec - now.tv_sec) * 1000000000LL
                    + (s_timer.next_time.tv_nsec - now.tv_nsec);

    if (diff_ns <= 0) {
        /* We're behind - re-anchor to next aligned boundary from now */
        uint64_t now_ns = (uint64_t)now.tv_sec * 1000000000ULL + now.tv_nsec;
        uint64_t period = s_timer.frame_period_ns;
        uint64_t next_ns = ((now_ns / period) + 1) * period;
        s_timer.next_time.tv_sec = (time_t)(next_ns / 1000000000ULL);
        s_timer.next_time.tv_nsec = (long)(next_ns % 1000000000ULL);
    }

    esp_eth_clock_set_target_time(CLOCK_PTP_SYSTEM, &s_timer.next_time);

    return false;
}

esp_err_t aes67_hw_timer_init(uint32_t frame_period_ns,
                               aes67_hw_timer_cb_t cb, void *user_data)
{
    if (!cb || frame_period_ns == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_timer, 0, sizeof(s_timer));
    s_timer.frame_period_ns = frame_period_ns;
    s_timer.cb = cb;
    s_timer.user_data = user_data;
    s_timer.running = false;

    /* Register the ISR callback with the EMAC PTP target time mechanism */
    int ret = esp_eth_clock_register_target_cb(CLOCK_PTP_SYSTEM, hw_timer_isr);
    if (ret != 0) {
        ESP_LOGE(TAG, "failed to register target time callback: %d", ret);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "initialized (period=%lu ns)", (unsigned long)frame_period_ns);
    return ESP_OK;
}

esp_err_t aes67_hw_timer_start(void)
{
    /* Get current PTP time and align to the next frame boundary */
    struct timespec now;
    if (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &now) != 0) {
        ESP_LOGE(TAG, "failed to get PTP time");
        return ESP_FAIL;
    }

    /* Align to next frame boundary: round up to nearest frame_period_ns */
    uint64_t now_ns = (uint64_t)now.tv_sec * 1000000000ULL + now.tv_nsec;
    uint64_t period = s_timer.frame_period_ns;
    uint64_t next_ns = ((now_ns / period) + 1) * period;

    s_timer.next_time.tv_sec = (time_t)(next_ns / 1000000000ULL);
    s_timer.next_time.tv_nsec = (long)(next_ns % 1000000000ULL);
    s_timer.running = true;

    /* Program the first target time */
    esp_eth_clock_set_target_time(CLOCK_PTP_SYSTEM, &s_timer.next_time);

    ESP_LOGI(TAG, "started (first target: %lld.%09ld)",
             (long long)s_timer.next_time.tv_sec,
             s_timer.next_time.tv_nsec);
    return ESP_OK;
}

esp_err_t aes67_hw_timer_stop(void)
{
    s_timer.running = false;
    ESP_LOGI(TAG, "stopped");
    return ESP_OK;
}
