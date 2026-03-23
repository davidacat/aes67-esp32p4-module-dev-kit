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
#include "aes67_config.h"
#include "aes67_convert.h"
#include "aes67_session.h"
#include "aes67_rtp.h"
#include "aes67_sap.h"
#include "aes67_audio.h"
#include "dsps_tone_gen.h"
#include "driver/i2c.h"
#include "freertos/stream_buffer.h"
#include "esp_cache.h"
#include "driver/i2s_std.h"
#include "es8311.h"

/* Session handle for SAP callback */
static aes67_session_handle_t s_session = NULL;
static bool s_sink_added = false;

/* Stream params for the Ethernet RTP hook (defined below, used by SAP callback) */
extern uint8_t s_hook_word_length;
extern uint8_t s_hook_channels;

/* SAP discovery callback: auto-subscribe to discovered remote sources */
static void sap_discovery_cb(bool is_announce,
                              const aes67_sap_remote_source_t *source,
                              void *user_data)
{
    if (!is_announce || !source || s_sink_added || !s_session) {
        return;
    }

    /* Subscribe to any discovered source (first one wins) */

    ESP_LOGI("main", "Auto-subscribing to \"%s\" from SAP", source->name);

    uint8_t sink_id = 0;
    esp_err_t err = aes67_session_add_sink(s_session, source->name,
                                            source->sdp, &sink_id);
    if (err == ESP_OK) {
        s_sink_added = true;
        /* Update Ethernet hook params from the SDP */
        aes67_sink_t sink;
        if (aes67_session_get_sink(s_session, sink_id, &sink) == ESP_OK) {
            s_hook_word_length = sink.rtp_config.word_length;
            s_hook_channels = sink.rtp_config.channels;
            ESP_LOGI("main", "Hook: wl=%u ch=%u", s_hook_word_length, s_hook_channels);
        }
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

/* I2S to ES8311 codec.
 * DOUT = GPIO 9: data FROM ESP32 TO ES8311 (DSDIN pin on codec)
 * DIN  = GPIO 11: data FROM ES8311 TO ESP32 (ASDOUT pin on codec)
 * See ESP-IDF issue #14297 for the DI/DO swap fix. */
#define I2S_MCLK_GPIO       13
#define I2S_SCLK_GPIO       12
#define I2S_LRCK_GPIO       10
#define I2S_DOUT_GPIO        9   /* ESP32 output -> codec DSDIN */
#define I2S_DIN_GPIO        11   /* codec ASDOUT -> ESP32 input */

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

/* --- Ethernet RTP hook: intercept RTP frames before lwIP --- */

static esp_err_t (*s_original_input)(esp_eth_handle_t, uint8_t*, uint32_t, void*, void*) = NULL;
static void *s_original_priv = NULL;
static i2s_chan_handle_t s_hook_tx_chan = NULL;
static int32_t *s_hook_tmp32 = NULL;
static int16_t *s_hook_i16 = NULL;
static uint32_t s_hook_pkt_count = 0;
static uint32_t s_hook_drop_count = 0;   /* Stream buffer full drops */
static uint32_t s_hook_fwd_count = 0;    /* Packets forwarded to lwIP (not RTP) */
static uint32_t s_hook_total_frames = 0; /* ALL Ethernet frames (before filtering) */
static uint16_t s_hook_last_seq = 0;     /* Last RTP sequence number */
static uint32_t s_hook_seq_gaps = 0;     /* Sequence discontinuities */
static uint32_t s_hook_seq_lost = 0;     /* Total lost packets (from seq gaps) */
StreamBufferHandle_t s_hook_sbuf = NULL;  /* Global: shared with playback task */
uint8_t s_hook_word_length = 3;          /* Default L24. Updated from SDP. */
uint8_t s_hook_channels = 2;             /* Default stereo. Updated from SDP. */

/* Called for EVERY Ethernet frame, before lwIP.
 * RTP multicast on port 5004: process + free. Everything else: forward to lwIP. */
static esp_err_t IRAM_ATTR eth_rtp_hook(esp_eth_handle_t eth_handle,
                                          uint8_t *buffer, uint32_t length,
                                          void *priv, void *info)
{
    s_hook_total_frames++;

    /* Quick reject: minimum Ethernet+IP+UDP+RTP header = 54 bytes */
    if (length < 54 || !s_hook_tx_chan) goto forward;

    /* EtherType must be IPv4 */
    if (buffer[12] != 0x08 || buffer[13] != 0x00) goto forward;

    /* IP protocol must be UDP (17) */
    uint8_t ihl = (buffer[14] & 0x0F) * 4;
    if (buffer[14 + 9] != 17) goto forward;

    /* Must be IPv4 multicast destination (224.0.0.0/4) */
    if ((buffer[14 + 16] & 0xF0) != 0xE0) goto forward;

    /* UDP destination port must be 5004 */
    int udp_off = 14 + ihl;
    uint16_t dst_port = (buffer[udp_off + 2] << 8) | buffer[udp_off + 3];
    if (dst_port != 5004) goto forward;

    /* RTP version must be 2 */
    int rtp_off = udp_off + 8;
    if ((buffer[rtp_off] >> 6) != 2) goto forward;

    /* Track RTP sequence numbers to detect packet loss */
    {
        uint16_t seq = (buffer[rtp_off + 2] << 8) | buffer[rtp_off + 3];
        if (s_hook_pkt_count > 0) {
            uint16_t expected = s_hook_last_seq + 1;
            if (seq != expected) {
                int16_t gap = (int16_t)(seq - expected);
                if (gap > 0 && gap < 100) {
                    s_hook_seq_gaps++;
                    s_hook_seq_lost += gap;
                }
            }
        }
        s_hook_last_seq = seq;
    }

    /* Extract RTP payload */
    uint8_t cc = buffer[rtp_off] & 0x0F;
    int payload_off = rtp_off + 12 + cc * 4;

    /* Handle RTP header extension */
    if (buffer[rtp_off] & 0x10) {
        if ((int)length < payload_off + 4) goto forward;
        uint16_t ext_len = (buffer[payload_off + 2] << 8) | buffer[payload_off + 3];
        payload_off += 4 + ext_len * 4;
    }

    if (payload_off >= (int)length) goto forward;

    const uint8_t *payload = buffer + payload_off;
    int payload_len = length - payload_off;

    /* ESP-IDF Ethernet driver already does M2C cache sync in
     * emac_esp_dma_receive_frame(). No additional sync needed. */

    /* Handle RTP padding (P bit) */
    if (buffer[rtp_off] & 0x20) {
        uint8_t pad_len = buffer[length - 1];
        payload_len -= pad_len;
        if (payload_len <= 0) goto forward;
    }

    /* Use UDP length field instead of Ethernet frame length to avoid
     * including Ethernet FCS or padding bytes in the audio data.
     * UDP length = header(8) + payload. */
    uint16_t udp_len = (buffer[udp_off + 4] << 8) | buffer[udp_off + 5];
    int rtp_total = udp_len - 8;  /* UDP payload = RTP header + RTP payload */
    int rtp_payload = rtp_total - (payload_off - rtp_off);
    if (rtp_payload > 0 && rtp_payload < payload_len) {
        payload_len = rtp_payload;  /* Trim to actual RTP payload */
    }

    /* Use word length and channel count from SDP (set when sink is added) */
    int wl = s_hook_word_length;
    int ch = s_hook_channels;
    int frame_bytes = ch * wl;
    int frames = payload_len / frame_bytes;
    if (frames <= 0 || frames > AES67_MAX_TIC_FRAMES) goto forward;

    /* Log first packet details */
    if (s_hook_pkt_count == 0) {
        ESP_DRAM_LOGI("eth_hook", "FIRST PKT: payload=%d bytes, wl=%d, ch=%d, frames=%d, "
                      "write_bytes=%d", payload_len, wl, ch, frames,
                      frames * ch * (int)sizeof(int32_t));
    }

    /* Convert to int32 */
    aes67_convert_from_net(payload, s_hook_tmp32, frames, ch, wl);

    /* For mono speaker: duplicate left channel to all output slots */
    for (int i = 0; i < frames; i++) {
        int32_t left = s_hook_tmp32[i * ch];
        for (int c = 0; c < ch; c++) {
            s_hook_tmp32[i * ch + c] = left;
        }
    }

    /* Write int32 to the slot ring (read by DMA ISR) */
    extern void aes67_audio_slot_write(const void *data, uint32_t len);
    aes67_audio_slot_write(s_hook_tmp32, frames * ch * sizeof(int32_t));

    s_hook_pkt_count++;
    if ((s_hook_pkt_count % 5000) == 0) {
        ESP_LOGI("eth_hook", "RTP: %lu pkts, %lu drops, %lu fwd | "
                 "seq: %lu gaps, %lu lost | sb=%u",
                 (unsigned long)s_hook_pkt_count,
                 (unsigned long)s_hook_drop_count,
                 (unsigned long)s_hook_fwd_count,
                 (unsigned long)s_hook_seq_gaps,
                 (unsigned long)s_hook_seq_lost,
                 s_hook_sbuf ? (unsigned)xStreamBufferBytesAvailable(s_hook_sbuf) : 0);
    }

    free(buffer);  /* We consumed the frame */
    return ESP_OK;

forward:
    s_hook_fwd_count++;
    /* Not RTP -- forward to lwIP via esp_netif_receive */
    if (s_original_priv) {
        return esp_netif_receive((esp_netif_t *)s_original_priv, buffer, length, NULL);
    }
    free(buffer);
    return ESP_OK;
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
    mac_config.flags = ETH_MAC_FLAG_PIN_TO_CORE;  /* Pin to core 0 */
    mac_config.rx_task_prio = 24;  /* Highest: above lwIP (23), feeds audio ISR directly */
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

    /* Ethernet RTP hook: intercept RTP at MAC level before lwIP */
    s_original_priv = eth_netif;
    /* 192 frames * max channels covers the AES67 practical max (4ms at 48kHz) */
    s_hook_tmp32 = heap_caps_malloc(
        192 * AES67_MAX_CHANNELS_PER_STREAM * sizeof(int32_t),
        MALLOC_CAP_INTERNAL);
    s_hook_i16 = heap_caps_malloc(
        192 * AES67_MAX_CHANNELS_PER_STREAM * sizeof(int16_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    esp_eth_update_input_path_info(eth_handle, eth_rtp_hook, eth_netif);
    ESP_LOGI(TAG, "Ethernet RTP hook installed");

    /* Enable Ethernet flow control to prevent frame drops under load.
     * Without this, the EMAC drops ~8% of multicast frames at high
     * packet rates (1000 pps) because the DMA can't keep up. */
    bool flow_ctrl = true;
    esp_eth_ioctl(eth_handle, ETH_CMD_S_FLOW_CTRL, &flow_ctrl);

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
     * I2S uses MCLK_MULTIPLE_384 (matching official ESP-IDF example).
     * MCLK = 48000 * 384 = 18.432 MHz */
    es8311_clock_config_t clk_cfg = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = 48000 * 384,     /* 18.432 MHz */
        .sample_frequency = 48000,
    };

    /* 32-bit resolution -- int32 maps directly, no truncation */
    ret = es8311_init(codec, &clk_cfg, ES8311_RESOLUTION_32, ES8311_RESOLUTION_32);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = es8311_voice_volume_set(codec, 70, NULL);
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

    ESP_LOGI(TAG, "ES8311 codec initialized (48kHz, 16-bit, MCLK=18.432MHz, vol=100)");
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

    /* ES8311 codec init is deferred until AFTER the AES67 I2S driver starts.
     * The ES8311 needs MCLK to be running during initialization. */

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

    /* Create stream buffer BEFORE starting the node. The DMA ISR callback
     * reads directly from this buffer, bypassing the playback task.
     * 30 packets (~120ms at 4ms ptime) for jitter absorption.
     * Trigger=1: ISR always reads whatever is available. */
    {
        uint32_t pkt_bytes = 192 * aes67_cfg.audio.channels * sizeof(int32_t);
        s_hook_sbuf = xStreamBufferCreate(pkt_bytes * 30, 1);

        /* Wire the stream buffer to the audio driver's DMA ISR */
        aes67_audio_handle_t audio = NULL;
        aes67_node_get_audio(node, &audio);
        aes67_audio_set_isr_stream_buf(audio, s_hook_sbuf);
        s_hook_tx_chan = aes67_audio_get_tx_chan(audio);
        ESP_LOGI(TAG, "Ethernet hook -> stream buffer -> DMA ISR (zero-overhead)");
    }

    /* Start PTP sync, RTP engine, SAP announcements and audio I/O.
     * This enables I2S which starts MCLK generation. The DMA ISR
     * callback is registered during audio start. */
    ESP_ERROR_CHECK(aes67_node_start(node));

    /* Initialize ES8311 codec AFTER I2S is running (MCLK must be active).
     * This is the critical init order discovered through testing. */
    esp_err_t codec_ret = es8311_codec_init();
    if (codec_ret != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 codec init FAILED: %s", esp_err_to_name(codec_ret));
    } else {
        ESP_LOGI(TAG, "ES8311 codec initialized (MCLK running)");
    }

    /* Obtain the session manager handle from the node so we can add streams */
    aes67_session_handle_t session = NULL;
    ESP_ERROR_CHECK(aes67_node_get_session(node, &session));
    s_session = session;

    /* Register SAP callback to auto-subscribe to discovered sources.
     * This will add a sink for the first remote source found (e.g. SIENNA). */
    aes67_sap_handle_t sap = NULL;
    /* Get SAP handle from the node config - it's stored internally.
     * For now, register via the session manager's SAP handle. */
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

    /* Source stream disabled while raw UDP PCB occupies port 5004.
     * TODO: use separate ports for TX and RX, or share the raw PCB. */
    uint8_t source_id = 0;
    esp_err_t src_err = aes67_session_add_source(session, "ESP32-P4 AES67",
                                                   2, AES67_CODEC_L24, &source_id);
    if (src_err == ESP_OK) {
        ESP_LOGI(TAG, "AES67 source stream active (id=%u, 2ch L24 @ 48kHz)", source_id);
    } else {
        ESP_LOGW(TAG, "Source stream skipped (port conflict with raw PCB): %s",
                 esp_err_to_name(src_err));
    }

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

    ESP_LOGI(TAG, "node ready (TX source disabled for RX throughput test)");

    /* Log stats every 2 seconds */
    extern void aes67_audio_log_isr_stats(void);
    uint32_t last_total = 0, last_rtp = 0, last_fwd = 0;
    TickType_t last_tick = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        aes67_audio_log_isr_stats();

        uint32_t now_total = s_hook_total_frames;
        uint32_t now_rtp = s_hook_pkt_count;
        uint32_t now_fwd = s_hook_fwd_count;
        TickType_t now_tick = xTaskGetTickCount();
        if (last_tick > 0) {
            uint32_t dt_ms = (now_tick - last_tick) * portTICK_PERIOD_MS;
            if (dt_ms > 0) {
                uint32_t total_fps = (now_total - last_total) * 1000UL / dt_ms;
                uint32_t rtp_pps = (now_rtp - last_rtp) * 1000UL / dt_ms;
                uint32_t fwd_pps = (now_fwd - last_fwd) * 1000UL / dt_ms;
                ESP_LOGI("eth_hook", "ALL=%lu/s RTP=%lu/s fwd=%lu/s",
                         (unsigned long)total_fps,
                         (unsigned long)rtp_pps,
                         (unsigned long)fwd_pps);
            }
        }
        last_total = now_total;
        last_rtp = now_rtp;
        last_fwd = now_fwd;
        last_tick = now_tick;
    }
}
