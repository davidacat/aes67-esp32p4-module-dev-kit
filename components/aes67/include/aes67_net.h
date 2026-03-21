/*
 * Network utilities for AES67.
 *
 * Multicast socket management, IGMP join/leave, IP address helpers.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create a UDP socket bound to the given port with multicast support.
 *
 * @param port      Local port to bind
 * @param reuse     Allow address reuse
 * @return Socket fd on success, -1 on error
 */
int aes67_net_create_udp_socket(uint16_t port, bool reuse);

/**
 * Join a multicast group on the given socket.
 *
 * @param sock          Socket fd
 * @param mcast_addr    Multicast group address string (e.g. "239.69.0.1")
 * @param if_addr       Local interface address string (NULL for default)
 * @return ESP_OK on success
 */
esp_err_t aes67_net_join_multicast(int sock, const char *mcast_addr,
                                   const char *if_addr);

/**
 * Leave a multicast group.
 */
esp_err_t aes67_net_leave_multicast(int sock, const char *mcast_addr,
                                    const char *if_addr);

/**
 * Set multicast TTL on a socket.
 */
esp_err_t aes67_net_set_multicast_ttl(int sock, uint8_t ttl);

/**
 * Set DSCP/TOS on a socket.
 */
esp_err_t aes67_net_set_dscp(int sock, uint8_t dscp);

/**
 * Set socket send/receive buffer sizes.
 */
esp_err_t aes67_net_set_buffers(int sock, int send_size, int recv_size);

/**
 * Parse an IPv4 address string to network byte order uint32_t.
 */
uint32_t aes67_net_ip_to_u32(const char *ip_str);

/**
 * Format a network byte order IPv4 address to string.
 */
void aes67_net_u32_to_ip(uint32_t ip, char *buf, size_t len);

/**
 * Check if an IP address is multicast (224.0.0.0/4).
 */
bool aes67_net_is_multicast(uint32_t ip);

/**
 * Get the local interface IP address for the default Ethernet interface.
 */
esp_err_t aes67_net_get_local_ip(uint32_t *ip);

/**
 * Get the local MAC address.
 */
esp_err_t aes67_net_get_local_mac(uint8_t mac[6]);

/**
 * Calculate multicast MAC address from multicast IP (IANA mapping).
 */
void aes67_net_mcast_ip_to_mac(uint32_t mcast_ip, uint8_t mac[6]);

#ifdef __cplusplus
}
#endif
