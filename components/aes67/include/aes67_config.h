/*
 * AES67 configuration structures.
 *
 * Kconfig provides compile-time defaults; these structs allow runtime
 * override before node initialization.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_eth.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Audio codec identifiers matching AES67/RAVENNA conventions */
typedef enum {
    AES67_CODEC_L16 = 0,   /* 16-bit linear PCM */
    AES67_CODEC_L24,        /* 24-bit linear PCM */
    AES67_CODEC_L32,        /* 32-bit linear PCM */
    AES67_CODEC_AM824,      /* IEC 61937 / AM824 */
} aes67_codec_t;

/* Network configuration */
typedef struct {
    char interface_name[16];            /* Ethernet interface, e.g. "ETH_0" */
    esp_eth_handle_t eth_handle;        /* Ethernet driver handle */
    char rtp_mcast_base[16];            /* Base multicast address for RTP */
    uint16_t rtp_port;                  /* RTP port (default 5004) */
    uint8_t ttl;                        /* Multicast TTL */
    uint8_t dscp;                       /* DSCP for QoS (default 46 = EF) */
} aes67_net_config_t;

/* PTP configuration */
typedef struct {
    uint8_t domain;                     /* PTP domain 0-127 */
    uint8_t dscp;                       /* DSCP for PTP packets */
    bool prefer_grandmaster;            /* Become GM if no other master found */
} aes67_ptp_config_t;

/* Audio configuration */
typedef struct {
    uint32_t sample_rate;               /* Sample rate in Hz */
    uint8_t word_length;                /* Bytes per sample (2, 3, or 4) */
    uint16_t packet_time_us;            /* Packet time in microseconds */
    uint8_t channels;                   /* Number of audio channels */
    aes67_codec_t codec;                /* Audio codec */
} aes67_audio_config_t;

/* I2S pin configuration */
typedef struct {
    int mck_gpio;
    int bck_gpio;
    int ws_gpio;
    int dout_gpio;
    int din_gpio;
} aes67_i2s_pins_t;

/* Top-level node configuration */
typedef struct {
    char node_name[64];                 /* Node name for SDP/mDNS */
    aes67_net_config_t net;
    aes67_ptp_config_t ptp;
    aes67_audio_config_t audio;
    aes67_i2s_pins_t i2s_pins;
    uint16_t jitter_buffer_ms;          /* Jitter buffer depth */
    uint16_t ring_buffer_ms;            /* Audio ring buffer size */
    bool use_psram;                     /* Allocate large buffers in PSRAM */
    bool sap_enabled;                   /* Enable SAP announcements */
    bool mdns_enabled;                  /* Enable mDNS advertisement */
} aes67_config_t;

/* Initialize a config struct with Kconfig defaults */
#define AES67_CONFIG_DEFAULT() { \
    .node_name = CONFIG_AES67_NODE_NAME, \
    .net = { \
        .interface_name = "ETH_0", \
        .eth_handle = NULL, \
        .rtp_mcast_base = CONFIG_AES67_RTP_MCAST_BASE, \
        .rtp_port = CONFIG_AES67_RTP_PORT, \
        .ttl = 64, \
        .dscp = 46, \
    }, \
    .ptp = { \
        .domain = CONFIG_AES67_PTP_DOMAIN, \
        .dscp = 46, \
        .prefer_grandmaster = true, \
    }, \
    .audio = { \
        .sample_rate = CONFIG_AES67_DEFAULT_SAMPLE_RATE, \
        .word_length = CONFIG_AES67_DEFAULT_WORD_LENGTH, \
        .packet_time_us = CONFIG_AES67_DEFAULT_PACKET_TIME_US, \
        .channels = 2, \
        .codec = AES67_CODEC_L24, \
    }, \
    .i2s_pins = { \
        .mck_gpio = CONFIG_AES67_I2S_MCK_GPIO, \
        .bck_gpio = CONFIG_AES67_I2S_BCK_GPIO, \
        .ws_gpio = CONFIG_AES67_I2S_WS_GPIO, \
        .dout_gpio = CONFIG_AES67_I2S_DOUT_GPIO, \
        .din_gpio = CONFIG_AES67_I2S_DIN_GPIO, \
    }, \
    .jitter_buffer_ms = CONFIG_AES67_JITTER_BUFFER_MS, \
    .ring_buffer_ms = CONFIG_AES67_AUDIO_RING_BUFFER_MS, \
    .use_psram = true, \
    .sap_enabled = true, \
    .mdns_enabled = true, \
}

#ifdef __cplusplus
}
#endif
