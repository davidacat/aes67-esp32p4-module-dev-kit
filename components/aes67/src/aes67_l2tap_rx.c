/*
 * L2 TAP based RTP receiver.
 *
 * Reads raw Ethernet frames from the L2 TAP interface, parses
 * Ethernet/IPv4/UDP/RTP headers, converts L24 audio to int16 mono,
 * and writes to I2S with DMA-paced blocking. This bypasses lwIP
 * entirely for minimum-latency audio reception.
 */

#include "aes67_l2tap_rx.h"
#include "aes67_convert.h"

#include <string.h>
#include <inttypes.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_vfs_l2tap.h"
#include "driver/i2s_common.h"

static const char *TAG = "l2tap_rx";

/* Ethernet header size (dst MAC + src MAC + EtherType) */
#define ETH_HDR_LEN        14
/* Minimum IPv4 header (no options) */
#define IPV4_HDR_LEN       20
/* UDP header size */
#define UDP_HDR_LEN        8
/* RTP header size (no CSRC, no extensions) */
#define RTP_HDR_LEN        12

/* Maximum Ethernet frame we expect (standard MTU) */
#define MAX_FRAME_LEN      1600

/* Offsets within the Ethernet frame */
#define OFF_ETHERTYPE      12
#define OFF_IP             ETH_HDR_LEN
#define OFF_IP_PROTOCOL    (OFF_IP + 9)
#define OFF_IP_DST         (OFF_IP + 16)
#define OFF_UDP            (OFF_IP + IPV4_HDR_LEN)
#define OFF_UDP_DST_PORT   (OFF_UDP + 2)
#define OFF_RTP            (OFF_UDP + UDP_HDR_LEN)
#define OFF_RTP_PAYLOAD    (OFF_RTP + RTP_HDR_LEN)

/* Maximum frames per RTP packet (48kHz * 4ms = 192) */
#define MAX_FRAMES_PER_PKT 192
/* Stereo L24: 2 channels * 3 bytes = 6 bytes per frame */
#define L24_FRAME_BYTES    6
/* Stereo channels */
#define NUM_CHANNELS       2

/* Task parameters */
#define L2TAP_RX_TASK_PRIO    23
#define L2TAP_RX_TASK_CORE    0
#define L2TAP_RX_TASK_STACK   8192

/* Context passed to the receiver task */
typedef struct {
    i2s_chan_handle_t tx_chan;
    uint16_t         rtp_port;
} l2tap_rx_ctx_t;

/*
 * Read a big-endian uint16 from a byte pointer.
 */
static inline IRAM_ATTR uint16_t read_be16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

/*
 * Check if an IPv4 address (in network byte order) is multicast (224.0.0.0/4).
 * First byte in the range 224-239.
 */
static inline IRAM_ATTR bool is_ipv4_multicast(const uint8_t *ip)
{
    return (ip[0] & 0xF0) == 0xE0;
}

/*
 * Parse and validate a raw Ethernet frame as an RTP packet.
 * Returns pointer to the RTP payload and sets *payload_len,
 * or NULL if the frame does not match our filter criteria.
 */
static IRAM_ATTR const uint8_t *parse_rtp_frame(
    const uint8_t *frame, int frame_len,
    uint16_t rtp_port, int *payload_len)
{
    /* Minimum frame: ETH + IP + UDP + RTP header */
    if (frame_len < OFF_RTP_PAYLOAD) {
        return NULL;
    }

    /* EtherType must be IPv4 (0x0800) */
    if (read_be16(frame + OFF_ETHERTYPE) != 0x0800) {
        return NULL;
    }

    /* IP protocol must be UDP (17) */
    if (frame[OFF_IP_PROTOCOL] != 17) {
        return NULL;
    }

    /* Must be IPv4 multicast destination */
    if (!is_ipv4_multicast(frame + OFF_IP_DST)) {
        return NULL;
    }

    /* Check IHL for actual IP header length (could have options) */
    uint8_t ihl = (frame[OFF_IP] & 0x0F) * 4;
    if (ihl < IPV4_HDR_LEN) {
        return NULL;
    }

    /* Recalculate offsets if IP header has options */
    int udp_off = OFF_IP + ihl;
    int rtp_off = udp_off + UDP_HDR_LEN;
    int payload_off = rtp_off + RTP_HDR_LEN;

    if (frame_len < payload_off) {
        return NULL;
    }

    /* UDP destination port must match our RTP port */
    uint16_t dst_port = read_be16(frame + udp_off + 2);
    if (dst_port != rtp_port) {
        return NULL;
    }

    /* Validate RTP version (must be 2) */
    uint8_t rtp_flags = frame[rtp_off];
    if ((rtp_flags >> 6) != 2) {
        return NULL;
    }

    /* Account for CSRC list if present */
    uint8_t cc = rtp_flags & 0x0F;
    payload_off += cc * 4;

    /* Account for RTP header extension if present */
    if (rtp_flags & 0x10) {
        int ext_off = rtp_off + RTP_HDR_LEN + cc * 4;
        if (frame_len < ext_off + 4) {
            return NULL;
        }
        uint16_t ext_len = read_be16(frame + ext_off + 2);
        payload_off += 4 + ext_len * 4;
    }

    if (payload_off >= frame_len) {
        return NULL;
    }

    *payload_len = frame_len - payload_off;
    return frame + payload_off;
}

/*
 * Convert L24 stereo RTP payload to int16 mono.
 *
 * Steps:
 *   1. Convert big-endian packed L24 to native int32 (via aes67_convert_from_net)
 *   2. Mono mix: average L and R channels
 *   3. Shift int32 down to int16
 */
static IRAM_ATTR void l24_stereo_to_i16_mono(
    const uint8_t *payload, int payload_len,
    int32_t *tmp32, int16_t *out16, int *out_frames)
{
    /* L24 stereo: 3 bytes * 2 channels = 6 bytes per frame */
    int frame_count = payload_len / L24_FRAME_BYTES;
    if (frame_count > MAX_FRAMES_PER_PKT) {
        frame_count = MAX_FRAMES_PER_PKT;
    }

    /* Unpack big-endian L24 to native int32, interleaved stereo */
    aes67_convert_from_net(payload, tmp32, frame_count, NUM_CHANNELS, 3);

    /* Mono mix (L+R)/2 and convert int32 -> stereo int16 (L=R) */
    for (int i = 0; i < frame_count; i++) {
        int32_t left  = tmp32[i * 2];
        int32_t right = tmp32[i * 2 + 1];
        int32_t mono  = (left >> 1) + (right >> 1);
        int16_t s16   = (int16_t)(mono >> 16);
        out16[i * 2]     = s16;  /* Left channel */
        out16[i * 2 + 1] = s16;  /* Right channel (same as left for mono) */
    }

    *out_frames = frame_count;
}

/*
 * Open and configure the L2 TAP file descriptor for the ETH_0 interface.
 * Returns the fd on success, -1 on failure.
 */
static int l2tap_open(void)
{
    int fd = open("/dev/net/tap", O_NONBLOCK);
    if (fd < 0) {
        ESP_LOGE(TAG, "failed to open /dev/net/tap: errno=%d", errno);
        return -1;
    }

    /* Bind to the ETH_0 interface */
    const char *ifname = "ETH_0";
    if (ioctl(fd, L2TAP_S_INTF_DEVICE, ifname) < 0) {
        ESP_LOGE(TAG, "L2TAP_S_INTF_DEVICE failed: errno=%d", errno);
        close(fd);
        return -1;
    }

    /* Filter for IPv4 frames only (EtherType 0x0800) */
    uint16_t eth_type_filter = 0x0800;
    if (ioctl(fd, L2TAP_S_RCV_FILTER, &eth_type_filter) < 0) {
        ESP_LOGE(TAG, "L2TAP_S_RCV_FILTER failed: errno=%d", errno);
        close(fd);
        return -1;
    }

    ESP_LOGI(TAG, "L2 TAP fd=%d opened on %s, filter=0x%04x",
             fd, ifname, eth_type_filter);
    return fd;
}

/*
 * Main L2 TAP receiver task. Reads raw Ethernet frames, filters for
 * RTP multicast packets, converts audio, and writes to I2S.
 */
static void l2tap_rx_task(void *arg)
{
    l2tap_rx_ctx_t *ctx = (l2tap_rx_ctx_t *)arg;

    /* Open L2 TAP interface */
    int fd = l2tap_open();
    if (fd < 0) {
        ESP_LOGE(TAG, "L2 TAP open failed, task exiting");
        heap_caps_free(ctx);
        vTaskDelete(NULL);
        return;
    }

    /* Allocate buffers from internal SRAM for cache-coherent DMA access */
    uint8_t *frame_buf = heap_caps_malloc(MAX_FRAME_LEN, MALLOC_CAP_INTERNAL);
    int32_t *tmp32_buf = heap_caps_malloc(
        MAX_FRAMES_PER_PKT * NUM_CHANNELS * sizeof(int32_t), MALLOC_CAP_INTERNAL);
    int16_t *pcm16_buf = heap_caps_malloc(
        MAX_FRAMES_PER_PKT * NUM_CHANNELS * sizeof(int16_t), MALLOC_CAP_INTERNAL);

    if (!frame_buf || !tmp32_buf || !pcm16_buf) {
        ESP_LOGE(TAG, "buffer allocation failed");
        goto cleanup;
    }

    ESP_LOGI(TAG, "receiver started, port=%u", ctx->rtp_port);

    uint32_t pkt_count = 0;

    while (1) {
        int len = read(fd, frame_buf, MAX_FRAME_LEN);
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                /* No data available, yield briefly */
                vTaskDelay(1);
                continue;
            }
            ESP_LOGE(TAG, "read error: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (len == 0) {
            vTaskDelay(1);
            continue;
        }

        /* Parse and filter the frame */
        int payload_len = 0;
        const uint8_t *payload = parse_rtp_frame(
            frame_buf, len, ctx->rtp_port, &payload_len);

        if (!payload || payload_len < L24_FRAME_BYTES) {
            continue;
        }

        /* Convert L24 stereo to int16 mono */
        int frame_count = 0;
        l24_stereo_to_i16_mono(payload, payload_len,
                               tmp32_buf, pcm16_buf, &frame_count);

        if (frame_count <= 0) {
            continue;
        }

        /* Write to I2S with DMA pacing (blocks until DMA descriptor is free).
         * This blocking write at portMAX_DELAY is the 48kHz pacing mechanism. */
        size_t bytes_written = 0;
        i2s_channel_write(ctx->tx_chan, pcm16_buf,
                          frame_count * NUM_CHANNELS * sizeof(int16_t),
                          &bytes_written, portMAX_DELAY);

        pkt_count++;
        if ((pkt_count % 5000) == 0) {
            ESP_LOGI(TAG, "rx packets: %"PRIu32, pkt_count);
        }
    }

cleanup:
    if (frame_buf)  heap_caps_free(frame_buf);
    if (tmp32_buf)  heap_caps_free(tmp32_buf);
    if (pcm16_buf)  heap_caps_free(pcm16_buf);
    if (fd >= 0)    close(fd);
    heap_caps_free(ctx);
    vTaskDelete(NULL);
}

void aes67_l2tap_rx_start(i2s_chan_handle_t tx_chan, uint16_t rtp_port)
{
    if (!tx_chan) {
        ESP_LOGE(TAG, "tx_chan is NULL");
        return;
    }

    l2tap_rx_ctx_t *ctx = heap_caps_malloc(sizeof(l2tap_rx_ctx_t),
                                           MALLOC_CAP_INTERNAL);
    if (!ctx) {
        ESP_LOGE(TAG, "failed to allocate context");
        return;
    }

    ctx->tx_chan  = tx_chan;
    ctx->rtp_port = rtp_port;

    BaseType_t ret = xTaskCreatePinnedToCore(
        l2tap_rx_task,
        "l2tap_rx",
        L2TAP_RX_TASK_STACK,
        ctx,
        L2TAP_RX_TASK_PRIO,
        NULL,
        L2TAP_RX_TASK_CORE);

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create l2tap_rx task");
        heap_caps_free(ctx);
        return;
    }

    ESP_LOGI(TAG, "L2 TAP RX task created (core %d, prio %d, port %u)",
             L2TAP_RX_TASK_CORE, L2TAP_RX_TASK_PRIO, rtp_port);
}
