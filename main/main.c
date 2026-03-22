/*
 * AES67 example application for ESP32-P4-NANO.
 *
 * Hardware configuration:
 *   - IP101 Ethernet PHY (RMII, external 50MHz crystal)
 *   - ES8311 audio codec (I2S, 48kHz, 24-bit)
 *   - PA enable on GPIO53
 *
 * This example initializes Ethernet, waits for an IP address, then
 * brings up the AES67 stack and creates a 2-channel L24 source stream
 * that captures audio from the ES8311 and streams it over RTP.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"

#include <math.h>
#include "aes67.h"
#include "aes67_session.h"
#include "aes67_rtp.h"
#include "aes67_sap.h"

static const char *TAG = "main";

/* Event group bit set when the Ethernet interface obtains an IP */
#define ETH_GOT_IP_BIT  BIT0

/* ---- Pin definitions for ESP32-P4-NANO ---- */

/* RMII Ethernet (directly wired, configured via emac_config) */
#define ETH_MDC_GPIO        31
#define ETH_MDIO_GPIO       52
#define ETH_REF_CLK_GPIO    50
#define ETH_PHY_RST_GPIO    51
#define ETH_PHY_ADDR        1

/* I2S to ES8311 codec */
#define I2S_MCLK_GPIO       13
#define I2S_SCLK_GPIO       12
#define I2S_LRCK_GPIO       10
#define I2S_DOUT_GPIO       11   /* ESP -> codec (ASDOUT) */
#define I2S_DIN_GPIO         9   /* codec -> ESP (DSDIN) */

/* Power amplifier enable (active high) */
#define PA_CTRL_GPIO        53

static EventGroupHandle_t s_eth_event_group;

/* Ethernet link event handler */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "ethernet link up");
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "ethernet link down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "ethernet started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "ethernet stopped");
        break;
    default:
        break;
    }
}

/* IP acquisition event handler */
static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_eth_event_group, ETH_GOT_IP_BIT);
    }
}

/*
 * Initialize the IP101 Ethernet PHY over RMII.
 * The ESP32-P4-NANO uses an external 50MHz crystal for the RMII reference
 * clock, so we configure EMAC_CLK_EXT_IN on GPIO50.
 */
static esp_err_t ethernet_init(esp_eth_handle_t *out_eth_handle)
{
    /* EMAC configuration with RMII pin mapping */
    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num = ETH_MDC_GPIO;
    emac_config.smi_gpio.mdio_num = ETH_MDIO_GPIO;
    emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac_config.clock_config.rmii.clock_gpio = ETH_REF_CLK_GPIO;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);
    if (!mac) {
        ESP_LOGE(TAG, "failed to create EMAC");
        return ESP_FAIL;
    }

    /* IP101 PHY at address 1, with hardware reset on GPIO51 */
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_PHY_RST_GPIO;
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);
    if (!phy) {
        ESP_LOGE(TAG, "failed to create IP101 PHY");
        return ESP_FAIL;
    }

    /* Install the Ethernet driver */
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    esp_err_t ret = esp_eth_driver_install(&eth_config, &eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ethernet driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create the netif with key "ETH_0" so the L2 TAP interface used by
     * ptpd can find it. The default ESP_NETIF_DEFAULT_ETH uses "ETH_DEF"
     * which does not match what ptpd_start("ETH_0") expects. */
    esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
    base_cfg.if_key = "ETH_0";
    esp_netif_config_t netif_config = {
        .base = &base_cfg,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    };
    esp_netif_t *eth_netif = esp_netif_new(&netif_config);
    if (!eth_netif) {
        ESP_LOGE(TAG, "failed to create ethernet netif");
        return ESP_FAIL;
    }

    /* Attach the driver to the netif via the default glue layer */
    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handle);
    ret = esp_netif_attach(eth_netif, glue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to attach ethernet to netif: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register event handlers for link and IP events */
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               &ip_event_handler, NULL));

    /* Start Ethernet */
    ret = esp_eth_start(eth_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ethernet start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    *out_eth_handle = eth_handle;
    return ESP_OK;
}

/* Enable the power amplifier on the ES8311 output stage */
static void pa_ctrl_enable(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PA_CTRL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PA_CTRL_GPIO, 1);
    ESP_LOGI(TAG, "PA enabled (GPIO%d high)", PA_CTRL_GPIO);
}

void app_main(void)
{
    /* Initialize NVS -- required by some ESP-IDF components internally */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Create the default event loop used by Ethernet and AES67 events */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Initialize the TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_init());

    /* Set up the event group used to synchronize on IP acquisition */
    s_eth_event_group = xEventGroupCreate();

    /* Bring up Ethernet with the IP101 PHY */
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(ethernet_init(&eth_handle));

    /* Block until we have an IP address. The AES67 stack needs network
     * connectivity for PTP, RTP multicast and SAP. */
    ESP_LOGI(TAG, "waiting for IP address...");
    xEventGroupWaitBits(s_eth_event_group, ETH_GOT_IP_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    /* Enable the power amplifier before starting audio */
    pa_ctrl_enable();

    /* Build the AES67 node configuration with our I2S pins and codec settings */
    aes67_config_t aes67_cfg = AES67_CONFIG_DEFAULT();
    strlcpy(aes67_cfg.node_name, "ESP32-P4 AES67", sizeof(aes67_cfg.node_name));

    aes67_cfg.net.eth_handle = eth_handle;

    aes67_cfg.i2s_pins.mck_gpio  = I2S_MCLK_GPIO;
    aes67_cfg.i2s_pins.bck_gpio  = I2S_SCLK_GPIO;
    aes67_cfg.i2s_pins.ws_gpio   = I2S_LRCK_GPIO;
    aes67_cfg.i2s_pins.dout_gpio = I2S_DOUT_GPIO;
    aes67_cfg.i2s_pins.din_gpio  = I2S_DIN_GPIO;

    aes67_cfg.audio.sample_rate   = 48000;
    aes67_cfg.audio.channels      = 2;
    aes67_cfg.audio.codec         = AES67_CODEC_L24;
    aes67_cfg.audio.word_length   = 3;       /* 24-bit = 3 bytes */
    aes67_cfg.audio.packet_time_us = 1000;   /* 1ms packet time per AES67 */

    /* Initialize the AES67 node */
    aes67_node_handle_t node = NULL;
    ESP_ERROR_CHECK(aes67_node_init(&aes67_cfg, &node));

    /* Start PTP sync, RTP engine, SAP announcements and audio I/O */
    ESP_ERROR_CHECK(aes67_node_start(node));

    /* Obtain the session manager handle from the node so we can add streams */
    aes67_session_handle_t session = NULL;
    ESP_ERROR_CHECK(aes67_node_get_session(node, &session));

    /* Create a 2-channel L24 source stream. The session manager generates
     * the SDP and begins SAP announcements automatically. */
    uint8_t source_id = 0;
    ESP_ERROR_CHECK(aes67_session_add_source(session, "ESP32-P4 AES67",
                                              2, AES67_CODEC_L24, &source_id));

    ESP_LOGI(TAG, "AES67 source stream active (id=%u, 2ch L24 @ 48kHz)", source_id);
    ESP_LOGI(TAG, "node ready -- streaming 1kHz test tone");

    /* Generate a 1kHz sine wave test tone into the source stream.
     * The audio frame callback handles I2S DMA, but we also inject
     * a software-generated tone directly into the RTP source buffer
     * so other AES67 devices can verify reception without needing
     * the ES8311 codec to be initialized via I2C. */

    aes67_source_t src_info;
    aes67_session_get_source(session, source_id, &src_info);

    const uint32_t sample_rate = 48000;
    const uint32_t tone_freq = 1000;
    const uint32_t samples_per_packet = (sample_rate * 1000) / 1000000; /* 48 for 1ms */
    const double amplitude = 0.5; /* -6 dBFS */
    const double two_pi = 2.0 * M_PI;

    uint32_t phase_sample = 0;
    int32_t tone_buf[samples_per_packet * 2]; /* stereo */

    while (1) {
        /* Fill one packet worth of stereo samples with 1kHz sine */
        for (uint32_t i = 0; i < samples_per_packet; i++) {
            double t = (double)phase_sample / (double)sample_rate;
            double val = amplitude * sin(two_pi * tone_freq * t);

            /* Convert to 24-bit left-justified in 32-bit (L24 format) */
            int32_t sample = (int32_t)(val * 8388607.0) << 8;
            tone_buf[i * 2 + 0] = sample;  /* left */
            tone_buf[i * 2 + 1] = sample;  /* right */

            phase_sample++;
            if (phase_sample >= sample_rate) {
                phase_sample = 0;
            }
        }

        /* Write the tone into the RTP source stream buffer */
        aes67_rtp_source_write(src_info.rtp_stream,
                               tone_buf, samples_per_packet);

        /* Pace at the packet rate (1ms) */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
