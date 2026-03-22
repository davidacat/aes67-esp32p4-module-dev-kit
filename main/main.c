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

#include <string.h>
#include "aes67.h"
#include "aes67_session.h"
#include "aes67_rtp.h"
#include "aes67_sap.h"
#include "aes67_audio.h"
#include "dsps_tone_gen.h"
#include "driver/i2c.h"
#include "es8311.h"

/* Session handle for SAP callback */
static aes67_session_handle_t s_session = NULL;
static bool s_sink_added = false;

/* SAP discovery callback: auto-subscribe to discovered remote sources */
static void sap_discovery_cb(bool is_announce,
                              const aes67_sap_remote_source_t *source,
                              void *user_data)
{
    if (!is_announce || !source || s_sink_added || !s_session) {
        return;
    }

    /* Subscribe to the first discovered source as a sink */
    ESP_LOGI("main", "Auto-subscribing to \"%s\" from SAP", source->name);

    uint8_t sink_id = 0;
    esp_err_t err = aes67_session_add_sink(s_session, source->name,
                                            source->sdp, &sink_id);
    if (err == ESP_OK) {
        s_sink_added = true;
        ESP_LOGI("main", "Sink added (id=%u) -- receiving \"%s\"",
                 sink_id, source->name);
    } else {
        ESP_LOGW("main", "Failed to add sink for \"%s\": %s",
                 source->name, esp_err_to_name(err));
    }
}

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

/* I2C for ES8311 codec control */
#define I2C_SCL_GPIO        8
#define I2C_SDA_GPIO        7
#define I2C_PORT            I2C_NUM_0
#define I2C_CLK_HZ          100000
#define ES8311_ADDR         ES8311_ADDRESS_0    /* 0x18, CE pin low */

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

/*
 * Initialize the ES8311 audio codec via I2C.
 * Configures it for 48kHz 24-bit I2S operation with both DAC (playback)
 * and ADC (capture) enabled.
 */
static esp_err_t es8311_codec_init(void)
{
    /* Initialize I2C master bus */
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLK_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_PORT, &i2c_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create ES8311 codec handle */
    es8311_handle_t codec = es8311_create(I2C_PORT, ES8311_ADDR);
    if (!codec) {
        ESP_LOGE(TAG, "ES8311 create failed");
        return ESP_FAIL;
    }

    /* Configure clock: MCLK from MCLK pin, 48kHz sample rate.
     * MCLK frequency = sample_rate * mclk_multiple.
     * We configured I2S with MCLK_MULTIPLE_384, so MCLK = 48000 * 384 = 18.432 MHz */
    es8311_clock_config_t clk_cfg = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = 48000 * 384,     /* 18.432 MHz */
        .sample_frequency = 48000,
    };

    ret = es8311_init(codec, &clk_cfg, ES8311_RESOLUTION_24, ES8311_RESOLUTION_24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set output volume (0-100) */
    ret = es8311_voice_volume_set(codec, 80, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ES8311 volume set failed: %s", esp_err_to_name(ret));
    }

    /* Unmute DAC output */
    ret = es8311_voice_mute(codec, false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ES8311 unmute failed: %s", esp_err_to_name(ret));
    }

    /* Configure microphone: analog mic, 24dB gain */
    es8311_microphone_config(codec, false);
    es8311_microphone_gain_set(codec, ES8311_MIC_GAIN_24DB);

    /* Dump ES8311 register state for debugging */
    es8311_register_dump(codec);

    ESP_LOGI(TAG, "ES8311 codec initialized (48kHz, 24-bit, vol=80, unmuted)");
    return ESP_OK;
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

    /* Initialize the ES8311 audio codec via I2C */
    esp_err_t codec_ret = es8311_codec_init();
    if (codec_ret != ESP_OK) {
        ESP_LOGW(TAG, "ES8311 codec init failed (audio may not work): %s",
                 esp_err_to_name(codec_ret));
    }

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
    s_session = session;

    /* Register SAP callback to auto-subscribe to discovered sources.
     * This will add a sink for the first remote source found (e.g. SIENNA). */
    aes67_sap_handle_t sap = NULL;
    /* Get SAP handle from the node config - it's stored internally.
     * For now, register via the session manager's SAP handle. */
    extern esp_err_t aes67_node_get_sap(aes67_node_handle_t handle,
                                         aes67_sap_handle_t *sap);
    if (aes67_node_get_sap(node, &sap) == ESP_OK && sap) {
        aes67_sap_register_cb(sap, sap_discovery_cb, NULL);
        ESP_LOGI(TAG, "SAP auto-subscribe enabled");

        /* Check if any sources were already discovered before callback
         * was registered (SAP starts during node_start). */
        int remote_count = aes67_sap_get_remote_count(sap);
        for (int i = 0; i < remote_count && !s_sink_added; i++) {
            aes67_sap_remote_source_t remote;
            if (aes67_sap_get_remote(sap, i, &remote) == ESP_OK) {
                sap_discovery_cb(true, &remote, NULL);
            }
        }
    }

    /* Create a 2-channel L24 source stream. The session manager generates
     * the SDP and begins SAP announcements automatically. */
    uint8_t source_id = 0;
    ESP_ERROR_CHECK(aes67_session_add_source(session, "ESP32-P4 AES67",
                                              2, AES67_CODEC_L24, &source_id));

    ESP_LOGI(TAG, "AES67 source stream active (id=%u, 2ch L24 @ 48kHz)", source_id);

    aes67_source_t src_info;
    aes67_session_get_source(session, source_id, &src_info);

    const uint32_t samples_per_packet = 48; /* 48kHz * 1ms */

    /* Use esp-dsp tone generator to precompute one full period of
     * a 1kHz sine wave at 48kHz into a float buffer, then convert
     * to 24-bit int32 once. The LUT is only 48 samples. */
    const uint32_t tone_period = 48; /* 48000 / 1000 */
    float tone_float[tone_period];
    dsps_tone_gen_f32(tone_float, tone_period, 0.5f,
                      1000.0f / 48000.0f, 0.0f);

    /* Convert float LUT to 24-bit left-justified int32 */
    int32_t sine_lut[tone_period];
    for (uint32_t n = 0; n < tone_period; n++) {
        sine_lut[n] = (int32_t)(tone_float[n] * 8388607.0f) << 8;
    }

    /* Stereo interleaved buffer for one packet */
    int32_t tone_buf[samples_per_packet * 2];
    uint32_t phase = 0;

    ESP_LOGI(TAG, "node ready -- streaming 1kHz test tone at -6 dBFS");

    /* Drop main task priority below the default so IDLE can run.
     * The tone generation is not time-critical. */
    vTaskPrioritySet(NULL, tskIDLE_PRIORITY + 1);

    while (1) {
        /* Fill packet from precomputed LUT (zero float math in loop) */
        for (uint32_t i = 0; i < samples_per_packet; i++) {
            int32_t sample = sine_lut[phase];
            tone_buf[i * 2 + 0] = sample;
            tone_buf[i * 2 + 1] = sample;
            phase++;
            if (phase >= tone_period) phase = 0;
        }

        aes67_rtp_source_write(src_info.rtp_stream,
                               tone_buf, samples_per_packet);

        /* 1ms pacing. At IDLE+1 priority the scheduler will preempt
         * us for IDLE task watchdog resets. */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
