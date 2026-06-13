/*
 * SPDX-FileCopyrightText: 2026 DatanoiseTV
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * Network utilities for AES67.
 *
 * Multicast socket management, IGMP, and IP helpers using lwIP on ESP-IDF.
 */

#include "aes67_net.h"

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#include "esp_log.h"
#include "esp_netif.h"

static const char *TAG = "aes67_net";

int aes67_net_create_udp_socket(uint16_t port, bool reuse)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket create failed: errno %d", errno);
        return -1;
    }

    if (reuse) {
        int opt = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind to port %u failed: errno %d", port, errno);
        close(sock);
        return -1;
    }

    return sock;
}

esp_err_t aes67_net_join_multicast(int sock, const char *mcast_addr,
                                   const char *if_addr)
{
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(mcast_addr);
    mreq.imr_interface.s_addr = if_addr ? inet_addr(if_addr) : htonl(INADDR_ANY);

    if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        ESP_LOGE(TAG, "join multicast %s failed: errno %d", mcast_addr, errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "joined multicast group %s", mcast_addr);
    return ESP_OK;
}

esp_err_t aes67_net_leave_multicast(int sock, const char *mcast_addr,
                                    const char *if_addr)
{
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(mcast_addr);
    mreq.imr_interface.s_addr = if_addr ? inet_addr(if_addr) : htonl(INADDR_ANY);

    if (setsockopt(sock, IPPROTO_IP, IP_DROP_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        ESP_LOGW(TAG, "leave multicast %s failed: errno %d", mcast_addr, errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t aes67_net_set_multicast_ttl(int sock, uint8_t ttl)
{
    int val = ttl;
    if (setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &val, sizeof(val)) < 0) {
        ESP_LOGE(TAG, "set multicast TTL failed: errno %d", errno);
        return ESP_FAIL;
    }

    /* Bind multicast output to the local interface so sendto doesn't
     * fail with EHOSTUNREACH when the default route changes. */
    uint32_t local_ip = 0;
    aes67_net_get_local_ip(&local_ip);
    struct in_addr mcast_if;
    mcast_if.s_addr = local_ip;
    if (setsockopt(sock, IPPROTO_IP, IP_MULTICAST_IF, &mcast_if, sizeof(mcast_if)) < 0) {
        ESP_LOGW(TAG, "set IP_MULTICAST_IF failed: errno %d", errno);
    }

    return ESP_OK;
}

esp_err_t aes67_net_set_dscp(int sock, uint8_t dscp)
{
    /* TOS field = DSCP << 2 */
    int tos = dscp << 2;
    if (setsockopt(sock, IPPROTO_IP, IP_TOS, &tos, sizeof(tos)) < 0) {
        ESP_LOGE(TAG, "set DSCP failed: errno %d", errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t aes67_net_set_buffers(int sock, int send_size, int recv_size)
{
    if (send_size > 0) {
        setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &send_size, sizeof(send_size));
    }
    if (recv_size > 0) {
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recv_size, sizeof(recv_size));
    }
    return ESP_OK;
}

uint32_t aes67_net_ip_to_u32(const char *ip_str)
{
    return inet_addr(ip_str);
}

void aes67_net_u32_to_ip(uint32_t ip, char *buf, size_t len)
{
    struct in_addr addr = { .s_addr = ip };
    const char *str = inet_ntoa(addr);
    if (str) {
        strncpy(buf, str, len - 1);
        buf[len - 1] = '\0';
    }
}

bool aes67_net_is_multicast(uint32_t ip)
{
    /* Multicast: 224.0.0.0 - 239.255.255.255 (0xE0000000 - 0xEFFFFFFF in host order) */
    uint32_t host_ip = ntohl(ip);
    return (host_ip & 0xF0000000) == 0xE0000000;
}

esp_err_t aes67_net_get_local_ip(uint32_t *ip)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("ETH_0");
    if (!netif) {
        ESP_LOGE(TAG, "no ethernet netif found");
        return ESP_ERR_NOT_FOUND;
    }

    esp_netif_ip_info_t ip_info;
    esp_err_t err = esp_netif_get_ip_info(netif, &ip_info);
    if (err != ESP_OK) {
        return err;
    }

    *ip = ip_info.ip.addr;
    return ESP_OK;
}

esp_err_t aes67_net_get_local_mac(uint8_t mac[6])
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("ETH_0");
    if (!netif) {
        return ESP_ERR_NOT_FOUND;
    }

    return esp_netif_get_mac(netif, mac);
}

void aes67_net_mcast_ip_to_mac(uint32_t mcast_ip, uint8_t mac[6])
{
    /* IANA multicast MAC: 01:00:5E:xx:xx:xx where lower 23 bits of IP */
    uint32_t ip_host = ntohl(mcast_ip);
    mac[0] = 0x01;
    mac[1] = 0x00;
    mac[2] = 0x5E;
    mac[3] = (ip_host >> 16) & 0x7F;
    mac[4] = (ip_host >> 8) & 0xFF;
    mac[5] = ip_host & 0xFF;
}
