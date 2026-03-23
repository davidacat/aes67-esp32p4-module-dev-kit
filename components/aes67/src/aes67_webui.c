/*
 * AES67 Web UI - HTTP server and WebSocket backend.
 *
 * Provides a REST API for device status and stream management,
 * serves an embedded gzipped SPA, and pushes real-time status
 * updates over WebSocket at 2Hz.
 *
 * All data reads use copy-out accessor functions from the node/session/
 * PTP/SAP subsystems. No direct struct access, no mutex contention
 * with audio paths.
 */

#include "aes67_webui.h"
#include "aes67.h"
#include "aes67_config.h"
#include "aes67_session.h"
#include "aes67_ptp.h"
#include "aes67_rtp.h"
#include "aes67_sap.h"
#include "aes67_net.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "aes67_webui";

/* Embedded gzipped index.html from EMBED_FILES */
extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[]   asm("_binary_index_html_gz_end");

/* JSON response buffer size for REST endpoints and WS pushes */
#define JSON_BUF_SIZE 2048

/* ---- Context ---- */

struct aes67_webui {
    aes67_node_handle_t node;
    httpd_handle_t      httpd;
    int                 ws_fd;      /* Active WebSocket fd (-1 if none) */
    TaskHandle_t        ws_task;    /* 2Hz status push task */
    uint16_t            port;
    bool                running;
};

/* ---- Codec name helper ---- */

static const char *codec_name(aes67_codec_t codec)
{
    switch (codec) {
    case AES67_CODEC_L16:   return "L16";
    case AES67_CODEC_L24:   return "L24";
    case AES67_CODEC_L32:   return "L32";
    case AES67_CODEC_AM824: return "AM824";
    default:                return "unknown";
    }
}

/* ---- PTP lock state string ---- */

static const char *ptp_lock_str(aes67_ptp_lock_state_t state)
{
    switch (state) {
    case AES67_PTP_LOCKED:   return "locked";
    case AES67_PTP_LOCKING:  return "locking";
    case AES67_PTP_UNLOCKED: return "unlocked";
    default:                 return "unknown";
    }
}

/* ---- GET / : serve gzipped SPA ---- */

static esp_err_t handler_index(httpd_req_t *req)
{
    size_t len = index_html_gz_end - index_html_gz_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, (const char *)index_html_gz_start, len);
}

/* ---- GET /favicon.ico : empty response to prevent 404 socket waste ---- */

static esp_err_t handler_favicon(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_set_hdr(req, "Connection", "close");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- GET /api/info ---- */

static esp_err_t handler_api_info(httpd_req_t *req)
{
    struct aes67_webui *ctx = (struct aes67_webui *)req->user_ctx;
    char *buf = malloc(JSON_BUF_SIZE);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    aes67_config_t cfg;
    aes67_node_get_config(ctx->node, &cfg);

    /* Uptime in seconds */
    int64_t uptime_us = esp_timer_get_time();
    uint32_t uptime_s = (uint32_t)(uptime_us / 1000000);

    /* Heap stats */
    uint32_t heap_free = esp_get_free_heap_size();
    uint32_t heap_min = esp_get_minimum_free_heap_size();

    /* Network info */
    uint32_t ip_u32 = 0;
    uint8_t mac[6] = {0};
    aes67_net_get_local_ip(&ip_u32);
    aes67_net_get_local_mac(mac);
    char ip_str[16];
    aes67_net_u32_to_ip(ip_u32, ip_str, sizeof(ip_str));

    int len = snprintf(buf, JSON_BUF_SIZE,
        "{"
        "\"name\":\"%s\","
        "\"version\":\"%d.%d.%d\","
        "\"ip\":\"%s\","
        "\"mac\":\"%02x:%02x:%02x:%02x:%02x:%02x\","
        "\"uptime\":%lu,"
        "\"heap_free\":%lu,"
        "\"heap_min\":%lu,"
        "\"sample_rate\":%lu,"
        "\"channels\":%u,"
        "\"codec\":\"%s\","
        "\"ptime_us\":%u"
        "}",
        cfg.node_name,
        AES67_VERSION_MAJOR, AES67_VERSION_MINOR, AES67_VERSION_PATCH,
        ip_str,
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
        (unsigned long)uptime_s,
        (unsigned long)heap_free,
        (unsigned long)heap_min,
        (unsigned long)cfg.audio.sample_rate,
        cfg.audio.channels,
        codec_name(cfg.audio.codec),
        cfg.audio.packet_time_us);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    free(buf);
    return ESP_OK;
}

/* ---- GET /api/ptp ---- */

static esp_err_t handler_api_ptp(httpd_req_t *req)
{
    struct aes67_webui *ctx = (struct aes67_webui *)req->user_ctx;
    char *buf = malloc(JSON_BUF_SIZE);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    aes67_ptp_handle_t ptp = NULL;
    aes67_node_get_ptp(ctx->node, &ptp);

    aes67_ptp_status_t st = {0};
    if (ptp) {
        aes67_ptp_get_status(ptp, &st);
    }

    /* Format grandmaster ID as dash-separated hex */
    char gm_str[24];
    snprintf(gm_str, sizeof(gm_str),
             "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
             st.grandmaster_id[0], st.grandmaster_id[1],
             st.grandmaster_id[2], st.grandmaster_id[3],
             st.grandmaster_id[4], st.grandmaster_id[5],
             st.grandmaster_id[6], st.grandmaster_id[7]);

    int len = snprintf(buf, JSON_BUF_SIZE,
        "{"
        "\"lock_state\":\"%s\","
        "\"grandmaster\":\"%s\","
        "\"domain\":%u,"
        "\"offset_ns\":%ld,"
        "\"jitter_ns\":%ld,"
        "\"path_delay_ns\":%ld"
        "}",
        ptp_lock_str(st.lock_state),
        gm_str,
        st.domain,
        (long)st.offset_ns,
        (long)st.jitter_ns,
        (long)st.path_delay_ns);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    free(buf);
    return ESP_OK;
}

/* ---- GET /api/sources ---- */

static esp_err_t handler_api_sources(httpd_req_t *req)
{
    struct aes67_webui *ctx = (struct aes67_webui *)req->user_ctx;
    char *buf = malloc(JSON_BUF_SIZE);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    aes67_session_handle_t sess = NULL;
    aes67_node_get_session(ctx->node, &sess);

    int pos = 0;
    buf[pos++] = '[';

    int count = sess ? aes67_session_get_source_count(sess) : 0;
    bool first = true;

    for (int i = 0; i < count && i < CONFIG_AES67_MAX_SOURCES; i++) {
        aes67_source_t src;
        if (aes67_session_get_source(sess, i, &src) != ESP_OK) {
            continue;
        }
        if (!src.enabled) {
            continue;
        }

        /* Get RTP stream stats if available */
        aes67_rtp_stream_status_t rtp_st = {0};
        if (src.rtp_stream) {
            aes67_rtp_stream_get_status(src.rtp_stream, &rtp_st);
        }

        if (!first) {
            buf[pos++] = ',';
        }
        first = false;

        pos += snprintf(buf + pos, JSON_BUF_SIZE - pos,
            "{"
            "\"id\":%u,"
            "\"name\":\"%s\","
            "\"channels\":%u,"
            "\"sample_rate\":%lu,"
            "\"codec\":\"%s\","
            "\"packets_sent\":%lu"
            "}",
            src.id, src.name,
            src.rtp_config.channels,
            (unsigned long)src.rtp_config.sample_rate,
            codec_name(src.rtp_config.codec),
            (unsigned long)rtp_st.packets_sent);

        if (pos >= JSON_BUF_SIZE - 128) {
            break;
        }
    }

    buf[pos++] = ']';
    buf[pos] = '\0';

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, pos);
    free(buf);
    return ESP_OK;
}

/* ---- GET /api/sinks ---- */

static esp_err_t handler_api_sinks(httpd_req_t *req)
{
    struct aes67_webui *ctx = (struct aes67_webui *)req->user_ctx;
    char *buf = malloc(JSON_BUF_SIZE);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    aes67_session_handle_t sess = NULL;
    aes67_node_get_session(ctx->node, &sess);

    int pos = 0;
    buf[pos++] = '[';

    int count = sess ? aes67_session_get_sink_count(sess) : 0;
    bool first = true;

    for (int i = 0; i < count && i < CONFIG_AES67_MAX_SINKS; i++) {
        aes67_sink_t sink;
        if (aes67_session_get_sink(sess, i, &sink) != ESP_OK) {
            continue;
        }
        if (!sink.enabled) {
            continue;
        }

        aes67_rtp_stream_status_t rtp_st = {0};
        if (sink.rtp_stream) {
            aes67_rtp_stream_get_status(sink.rtp_stream, &rtp_st);
        }

        if (!first) {
            buf[pos++] = ',';
        }
        first = false;

        pos += snprintf(buf + pos, JSON_BUF_SIZE - pos,
            "{"
            "\"id\":%u,"
            "\"name\":\"%s\","
            "\"channels\":%u,"
            "\"sample_rate\":%lu,"
            "\"codec\":\"%s\","
            "\"packets_received\":%lu,"
            "\"packets_lost\":%lu,"
            "\"jitter_us\":%ld,"
            "\"receiving\":%s"
            "}",
            sink.id, sink.name,
            sink.rtp_config.channels,
            (unsigned long)sink.rtp_config.sample_rate,
            codec_name(sink.rtp_config.codec),
            (unsigned long)rtp_st.packets_received,
            (unsigned long)rtp_st.packets_lost,
            (long)rtp_st.jitter_us,
            (rtp_st.status_flags & AES67_RTP_STATUS_RECEIVING) ? "true" : "false");

        if (pos >= JSON_BUF_SIZE - 128) {
            break;
        }
    }

    buf[pos++] = ']';
    buf[pos] = '\0';

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, pos);
    free(buf);
    return ESP_OK;
}

/* ---- GET /api/sap/remotes ---- */

static esp_err_t handler_api_sap_remotes(httpd_req_t *req)
{
    struct aes67_webui *ctx = (struct aes67_webui *)req->user_ctx;
    char *buf = malloc(JSON_BUF_SIZE);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    aes67_sap_handle_t sap = NULL;
    aes67_node_get_sap(ctx->node, &sap);

    int pos = 0;
    buf[pos++] = '[';

    int count = sap ? aes67_sap_get_remote_count(sap) : 0;
    bool first = true;

    for (int i = 0; i < count; i++) {
        aes67_sap_remote_source_t remote;
        if (aes67_sap_get_remote(sap, i, &remote) != ESP_OK) {
            continue;
        }

        if (!first) {
            buf[pos++] = ',';
        }
        first = false;

        char origin_str[16];
        aes67_net_u32_to_ip(remote.origin_ip, origin_str, sizeof(origin_str));

        pos += snprintf(buf + pos, JSON_BUF_SIZE - pos,
            "{"
            "\"name\":\"%s\","
            "\"origin\":\"%s\","
            "\"msg_id\":%u"
            "}",
            remote.name,
            origin_str,
            remote.msg_id);

        if (pos >= JSON_BUF_SIZE - 128) {
            break;
        }
    }

    buf[pos++] = ']';
    buf[pos] = '\0';

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, pos);
    free(buf);
    return ESP_OK;
}

/* ---- POST /api/sinks : subscribe to remote source ---- */

static esp_err_t handler_post_sink(httpd_req_t *req)
{
    struct aes67_webui *ctx = (struct aes67_webui *)req->user_ctx;

    /* Read request body (max 2KB for SDP) */
    int content_len = req->content_len;
    if (content_len <= 0 || content_len > JSON_BUF_SIZE) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body length");
        return ESP_FAIL;
    }

    char *body = malloc(content_len + 1);
    if (!body) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int received = httpd_req_recv(req, body, content_len);
    if (received != content_len) {
        free(body);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete body");
        return ESP_FAIL;
    }
    body[content_len] = '\0';

    /*
     * Minimal JSON parsing: extract "name" and "sdp" fields.
     * Expected format: {"name":"...","sdp":"v=0\r\n..."}
     */
    const char *name_key = "\"name\":\"";
    const char *sdp_key = "\"sdp\":\"";

    char name[64] = {0};
    char *sdp = NULL;

    /* Extract name */
    char *p = strstr(body, name_key);
    if (p) {
        p += strlen(name_key);
        char *end = strchr(p, '"');
        if (end) {
            size_t nlen = end - p;
            if (nlen >= sizeof(name)) nlen = sizeof(name) - 1;
            memcpy(name, p, nlen);
            name[nlen] = '\0';
        }
    }

    /* Extract SDP */
    p = strstr(body, sdp_key);
    if (p) {
        p += strlen(sdp_key);
        /* Find the closing quote (skip escaped quotes) */
        char *end = p;
        while (*end && !(*end == '"' && *(end - 1) != '\\')) {
            end++;
        }
        if (*end == '"') {
            size_t slen = end - p;
            sdp = malloc(slen + 1);
            if (sdp) {
                memcpy(sdp, p, slen);
                sdp[slen] = '\0';
                /* Unescape \r\n sequences */
                char *r = sdp, *w = sdp;
                while (*r) {
                    if (r[0] == '\\' && r[1] == 'r') {
                        *w++ = '\r';
                        r += 2;
                    } else if (r[0] == '\\' && r[1] == 'n') {
                        *w++ = '\n';
                        r += 2;
                    } else {
                        *w++ = *r++;
                    }
                }
                *w = '\0';
            }
        }
    }

    free(body);

    if (!name[0] || !sdp || !sdp[0]) {
        free(sdp);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "Missing 'name' or 'sdp' field");
        return ESP_FAIL;
    }

    /* Add the sink via session manager */
    aes67_session_handle_t sess = NULL;
    aes67_node_get_session(ctx->node, &sess);
    if (!sess) {
        free(sdp);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    uint8_t id = 0;
    esp_err_t ret = aes67_session_add_sink(sess, name, sdp, &id);
    free(sdp);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "add_sink failed: %s", esp_err_to_name(ret));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "Failed to add sink");
        return ESP_FAIL;
    }

    /* Respond with the assigned ID */
    char resp[32];
    int len = snprintf(resp, sizeof(resp), "{\"id\":%u}", id);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, len);

    ESP_LOGI(TAG, "Sink '%s' added with id=%u", name, id);
    return ESP_OK;
}

/* ---- DELETE /api/sinks/N : remove sink by ID ---- */

static esp_err_t handler_delete_sink(httpd_req_t *req)
{
    struct aes67_webui *ctx = (struct aes67_webui *)req->user_ctx;

    /* Parse sink ID from URI: /api/sinks/<id> */
    const char *uri = req->uri;
    const char *id_str = strrchr(uri, '/');
    if (!id_str || !*(id_str + 1)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing sink ID");
        return ESP_FAIL;
    }
    id_str++; /* skip the '/' */

    int id = atoi(id_str);
    if (id < 0 || id > 255) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid sink ID");
        return ESP_FAIL;
    }

    aes67_session_handle_t sess = NULL;
    aes67_node_get_session(ctx->node, &sess);
    if (!sess) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    esp_err_t ret = aes67_session_remove_sink(sess, (uint8_t)id);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "remove_sink(%d) failed: %s", id, esp_err_to_name(ret));
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Sink not found");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"ok\":true}", 11);

    ESP_LOGI(TAG, "Sink id=%d removed", id);
    return ESP_OK;
}

/* ---- WebSocket handler ---- */

static esp_err_t handler_ws(httpd_req_t *req)
{
    struct aes67_webui *ctx = (struct aes67_webui *)req->user_ctx;

    if (req->method == HTTP_GET) {
        /* WebSocket upgrade -- store the fd */
        ctx->ws_fd = httpd_req_to_sockfd(req);
        ESP_LOGI(TAG, "WebSocket connected (fd=%d)", ctx->ws_fd);
        return ESP_OK;
    }

    /* Handle incoming WS frames (e.g. pings, close) */
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    /* Read the frame to acknowledge it */
    uint8_t frame_buf[128];
    ws_pkt.payload = frame_buf;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(frame_buf));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WS recv failed: %s", esp_err_to_name(ret));
        ctx->ws_fd = -1;
    }

    return ESP_OK;
}

/* ---- WebSocket 2Hz push task ---- */

/*
 * Build a compact JSON status payload and push it to the connected
 * WebSocket client. Runs every 500ms on core 1, priority 3.
 */
static void ws_push_task(void *arg)
{
    struct aes67_webui *ctx = (struct aes67_webui *)arg;
    char *buf = malloc(JSON_BUF_SIZE);

    if (!buf) {
        ESP_LOGW(TAG, "WS push task: alloc failed");
        vTaskDelete(NULL);
        return;
    }

    while (ctx->running) {
        vTaskDelay(pdMS_TO_TICKS(500));

        /* Skip if no WebSocket client connected */
        if (ctx->ws_fd < 0 || !ctx->httpd) {
            continue;
        }

        /* Gather status data */
        int64_t uptime_us = esp_timer_get_time();
        uint32_t uptime_s = (uint32_t)(uptime_us / 1000000);
        uint32_t heap_free = esp_get_free_heap_size();

        /* PTP status */
        aes67_ptp_handle_t ptp = NULL;
        aes67_node_get_ptp(ctx->node, &ptp);
        aes67_ptp_status_t ptp_st = {0};
        if (ptp) {
            aes67_ptp_get_status(ptp, &ptp_st);
        }

        char gm_str[24];
        snprintf(gm_str, sizeof(gm_str),
                 "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X",
                 ptp_st.grandmaster_id[0], ptp_st.grandmaster_id[1],
                 ptp_st.grandmaster_id[2], ptp_st.grandmaster_id[3],
                 ptp_st.grandmaster_id[4], ptp_st.grandmaster_id[5],
                 ptp_st.grandmaster_id[6], ptp_st.grandmaster_id[7]);

        /* Start building JSON */
        int pos = snprintf(buf, JSON_BUF_SIZE,
            "{"
            "\"t\":\"status\","
            "\"uptime\":%lu,"
            "\"heap_free\":%lu,"
            "\"ptp\":{\"lock\":\"%s\",\"gm\":\"%s\",\"offset\":%ld,\"jitter\":%ld,\"delay\":%ld},",
            (unsigned long)uptime_s,
            (unsigned long)heap_free,
            ptp_lock_str(ptp_st.lock_state),
            gm_str,
            (long)ptp_st.offset_ns,
            (long)ptp_st.jitter_ns,
            (long)ptp_st.path_delay_ns);

        /* Source stats */
        aes67_session_handle_t sess = NULL;
        aes67_node_get_session(ctx->node, &sess);

        pos += snprintf(buf + pos, JSON_BUF_SIZE - pos, "\"sources\":[");
        if (sess) {
            bool first = true;
            int src_count = aes67_session_get_source_count(sess);
            for (int i = 0; i < src_count && i < CONFIG_AES67_MAX_SOURCES; i++) {
                aes67_source_t src;
                if (aes67_session_get_source(sess, i, &src) != ESP_OK || !src.enabled) {
                    continue;
                }
                aes67_rtp_stream_status_t rtp_st = {0};
                if (src.rtp_stream) {
                    aes67_rtp_stream_get_status(src.rtp_stream, &rtp_st);
                }
                if (!first) buf[pos++] = ',';
                first = false;
                const char *codec = aes67_codec_to_str(src.rtp_config.codec);
                pos += snprintf(buf + pos, JSON_BUF_SIZE - pos,
                    "{\"id\":%u,\"name\":\"%s\",\"codec\":\"%s\",\"rate\":%lu,"
                    "\"ch\":%u,\"ptime\":%u,\"tx\":%lu}",
                    src.id, src.name, codec,
                    (unsigned long)src.rtp_config.sample_rate,
                    src.rtp_config.channels,
                    src.rtp_config.packet_time_us,
                    (unsigned long)rtp_st.packets_sent);
                if (pos >= JSON_BUF_SIZE - 256) break;
            }
        }
        pos += snprintf(buf + pos, JSON_BUF_SIZE - pos, "],");

        /* Sink stats */
        pos += snprintf(buf + pos, JSON_BUF_SIZE - pos, "\"sinks\":[");
        if (sess) {
            bool first = true;
            int sink_count = aes67_session_get_sink_count(sess);
            for (int i = 0; i < sink_count && i < CONFIG_AES67_MAX_SINKS; i++) {
                aes67_sink_t sink;
                if (aes67_session_get_sink(sess, i, &sink) != ESP_OK || !sink.enabled) {
                    continue;
                }
                aes67_rtp_stream_status_t rtp_st = {0};
                if (sink.rtp_stream) {
                    aes67_rtp_stream_get_status(sink.rtp_stream, &rtp_st);
                }
                if (!first) buf[pos++] = ',';
                first = false;
                const char *scodec = aes67_codec_to_str(sink.rtp_config.codec);
                pos += snprintf(buf + pos, JSON_BUF_SIZE - pos,
                    "{\"id\":%u,\"name\":\"%s\",\"codec\":\"%s\",\"rate\":%lu,"
                    "\"ch\":%u,\"ptime\":%u,\"rx\":%lu,\"lost\":%lu,"
                    "\"jitter\":%ld,\"active\":%s}",
                    sink.id, sink.name, scodec,
                    (unsigned long)sink.rtp_config.sample_rate,
                    sink.rtp_config.channels,
                    sink.rtp_config.packet_time_us,
                    (unsigned long)rtp_st.packets_received,
                    (unsigned long)rtp_st.packets_lost,
                    (long)rtp_st.jitter_us,
                    (rtp_st.status_flags & AES67_RTP_STATUS_RECEIVING) ? "true" : "false");
                if (pos >= JSON_BUF_SIZE - 256) break;
            }
        }
        pos += snprintf(buf + pos, JSON_BUF_SIZE - pos, "],");

        /* SAP remotes */
        aes67_sap_handle_t sap = NULL;
        aes67_node_get_sap(ctx->node, &sap);

        pos += snprintf(buf + pos, JSON_BUF_SIZE - pos, "\"sap_remotes\":[");
        if (sap) {
            bool first = true;
            int rcount = aes67_sap_get_remote_count(sap);
            for (int i = 0; i < rcount; i++) {
                aes67_sap_remote_source_t remote;
                if (aes67_sap_get_remote(sap, i, &remote) != ESP_OK) {
                    continue;
                }
                char origin_str[16];
                aes67_net_u32_to_ip(remote.origin_ip, origin_str, sizeof(origin_str));
                if (!first) buf[pos++] = ',';
                first = false;
                pos += snprintf(buf + pos, JSON_BUF_SIZE - pos,
                    "{\"name\":\"%s\",\"origin\":\"%s\"}",
                    remote.name, origin_str);
                if (pos >= JSON_BUF_SIZE - 128) break;
            }
        }
        pos += snprintf(buf + pos, JSON_BUF_SIZE - pos, "]");

        /* Ethernet hook stats (defined in main.c) */
        extern uint32_t s_hook_pkt_count;
        extern uint32_t s_hook_seq_lost;
        extern uint32_t s_hook_total_frames;
        pos += snprintf(buf + pos, JSON_BUF_SIZE - pos,
            ",\"eth\":{\"rtp\":%lu,\"total\":%lu,\"lost\":%lu}",
            (unsigned long)s_hook_pkt_count,
            (unsigned long)s_hook_total_frames,
            (unsigned long)s_hook_seq_lost);

        /* Close the JSON object */
        buf[pos++] = '}';
        buf[pos] = '\0';

        /* Send via async WS frame (must be queued through httpd work queue) */
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(ws_pkt));
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;
        ws_pkt.payload = (uint8_t *)buf;
        ws_pkt.len = pos;

        esp_err_t ret = httpd_ws_send_frame_async(ctx->httpd, ctx->ws_fd, &ws_pkt);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "WS send failed (fd=%d): %s", ctx->ws_fd,
                     esp_err_to_name(ret));
            ctx->ws_fd = -1;
        }
    }

    free(buf);
    ESP_LOGI(TAG, "WS push task stopped");
    vTaskDelete(NULL);
}

/* ---- Public API ---- */

esp_err_t aes67_webui_init(aes67_node_handle_t node, uint16_t port,
                            aes67_webui_handle_t *handle)
{
    if (!node || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    struct aes67_webui *ctx = calloc(1, sizeof(struct aes67_webui));
    if (!ctx) {
        ESP_LOGW(TAG, "Failed to allocate webui context");
        return ESP_ERR_NO_MEM;
    }

    ctx->node = node;
    ctx->port = port;
    ctx->ws_fd = -1;
    ctx->ws_task = NULL;
    ctx->httpd = NULL;
    ctx->running = false;

    *handle = ctx;
    return ESP_OK;
}

esp_err_t aes67_webui_start(aes67_webui_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->running) {
        ESP_LOGW(TAG, "Web UI already running");
        return ESP_OK;
    }

    /* Configure and start the HTTP server */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = handle->port;
    config.max_uri_handlers = 16;
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    config.max_open_sockets = 7;
    config.keep_alive_enable = false;  /* Close HTTP conns immediately, free sockets for WS */
    config.core_id = 1;  /* Run HTTP server on core 1 (audio on core 0) */

    esp_err_t ret = httpd_start(&handle->httpd, &config);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Register URI handlers -- all use the webui context as user_ctx */

    /* Serve the SPA */
    const httpd_uri_t uri_index = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handler_index,
        .user_ctx = handle,
    };
    httpd_register_uri_handler(handle->httpd, &uri_index);

    /* Favicon -- prevent browser 404 and socket waste */
    const httpd_uri_t uri_favicon = {
        .uri = "/favicon.ico",
        .method = HTTP_GET,
        .handler = handler_favicon,
        .user_ctx = handle,
    };
    httpd_register_uri_handler(handle->httpd, &uri_favicon);

    /* WebSocket endpoint */
    const httpd_uri_t uri_ws = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = handler_ws,
        .user_ctx = handle,
        .is_websocket = true,
    };
    httpd_register_uri_handler(handle->httpd, &uri_ws);

    /* REST: device info */
    const httpd_uri_t uri_info = {
        .uri = "/api/info",
        .method = HTTP_GET,
        .handler = handler_api_info,
        .user_ctx = handle,
    };
    httpd_register_uri_handler(handle->httpd, &uri_info);

    /* REST: PTP status */
    const httpd_uri_t uri_ptp = {
        .uri = "/api/ptp",
        .method = HTTP_GET,
        .handler = handler_api_ptp,
        .user_ctx = handle,
    };
    httpd_register_uri_handler(handle->httpd, &uri_ptp);

    /* REST: source streams */
    const httpd_uri_t uri_sources = {
        .uri = "/api/sources",
        .method = HTTP_GET,
        .handler = handler_api_sources,
        .user_ctx = handle,
    };
    httpd_register_uri_handler(handle->httpd, &uri_sources);

    /* REST: sink streams (GET) */
    const httpd_uri_t uri_sinks_get = {
        .uri = "/api/sinks",
        .method = HTTP_GET,
        .handler = handler_api_sinks,
        .user_ctx = handle,
    };
    httpd_register_uri_handler(handle->httpd, &uri_sinks_get);

    /* REST: SAP discovered remotes */
    const httpd_uri_t uri_sap_remotes = {
        .uri = "/api/sap/remotes",
        .method = HTTP_GET,
        .handler = handler_api_sap_remotes,
        .user_ctx = handle,
    };
    httpd_register_uri_handler(handle->httpd, &uri_sap_remotes);

    /* REST: add sink (POST) */
    const httpd_uri_t uri_sinks_post = {
        .uri = "/api/sinks",
        .method = HTTP_POST,
        .handler = handler_post_sink,
        .user_ctx = handle,
    };
    httpd_register_uri_handler(handle->httpd, &uri_sinks_post);

    /* REST: remove sink (DELETE /api/sinks/N) */
    const httpd_uri_t uri_sinks_delete = {
        .uri = "/api/sinks/*",
        .method = HTTP_DELETE,
        .handler = handler_delete_sink,
        .user_ctx = handle,
    };
    httpd_register_uri_handler(handle->httpd, &uri_sinks_delete);

    /* Start the 2Hz WebSocket push task on core 1 */
    handle->running = true;
    BaseType_t xret = xTaskCreatePinnedToCore(
        ws_push_task, "aes67_ws", 4096, handle,
        3, &handle->ws_task, 1);
    if (xret != pdPASS) {
        ESP_LOGW(TAG, "Failed to create WS push task");
        handle->ws_task = NULL;
    }

    ESP_LOGI(TAG, "Web UI started on port %u", handle->port);
    return ESP_OK;
}

esp_err_t aes67_webui_stop(aes67_webui_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!handle->running) {
        return ESP_OK;
    }

    /* Signal the push task to exit */
    handle->running = false;

    /* Give it time to exit cleanly */
    if (handle->ws_task) {
        vTaskDelay(pdMS_TO_TICKS(600));
        handle->ws_task = NULL;
    }

    /* Stop the HTTP server */
    if (handle->httpd) {
        httpd_stop(handle->httpd);
        handle->httpd = NULL;
    }

    handle->ws_fd = -1;

    ESP_LOGI(TAG, "Web UI stopped");
    return ESP_OK;
}
