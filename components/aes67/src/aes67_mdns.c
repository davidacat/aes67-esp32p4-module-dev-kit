/*
 * mDNS service advertisement for AES67/RAVENNA.
 *
 * Advertises this node as an AES67 device using DNS-SD/mDNS,
 * making it discoverable by other AES67 devices on the network.
 */

#include "aes67_net.h"
#include "aes67_config.h"

#include <string.h>
#include "esp_log.h"
#include "mdns.h"

static const char *TAG = "aes67_mdns";

esp_err_t aes67_mdns_init(const char *node_name, uint16_t rtp_port)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = mdns_hostname_set(node_name);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_hostname_set failed: %s", esp_err_to_name(err));
        return err;
    }

    err = mdns_instance_name_set(node_name);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mdns_instance_name_set failed: %s", esp_err_to_name(err));
    }

    /* Advertise as RAVENNA/AES67 device */
    mdns_txt_item_t txt_items[] = {
        { "proto", "AES67" },
        { "rate",  "48000" },
        { "ch",    "2" },
    };

    err = mdns_service_add(node_name, "_ravenna", "_udp",
                           rtp_port, txt_items, 3);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_service_add _ravenna._udp failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    /* Also advertise the rtsp service for SDP retrieval */
    err = mdns_service_add(node_name, "_rtsp", "_tcp", 554, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mdns_service_add _rtsp._tcp failed (non-critical)");
    }

    ESP_LOGI(TAG, "mDNS services registered: %s", node_name);
    return ESP_OK;
}

esp_err_t aes67_mdns_update_txt(uint32_t sample_rate, uint8_t channels)
{
    char rate_str[12];
    char ch_str[4];
    snprintf(rate_str, sizeof(rate_str), "%lu", (unsigned long)sample_rate);
    snprintf(ch_str, sizeof(ch_str), "%u", channels);

    mdns_txt_item_t txt_items[] = {
        { "proto", "AES67" },
        { "rate",  rate_str },
        { "ch",    ch_str },
    };

    return mdns_service_txt_set("_ravenna", "_udp", txt_items, 3);
}

void aes67_mdns_deinit(void)
{
    mdns_service_remove_all();
    mdns_free();
    ESP_LOGI(TAG, "mDNS services removed");
}
