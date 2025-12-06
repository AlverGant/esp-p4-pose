/**
 * @file at_wifi.h
 * @brief WiFi management via ESP-AT commands
 */

#pragma once

#include "at_client.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// WiFi mode
typedef enum {
    AT_WIFI_MODE_NULL = 0,
    AT_WIFI_MODE_STA = 1,
    AT_WIFI_MODE_AP = 2,
    AT_WIFI_MODE_STA_AP = 3,
} at_wifi_mode_t;

// WiFi connection status
typedef enum {
    AT_WIFI_STATUS_UNKNOWN = 0,
    AT_WIFI_STATUS_CONNECTED,
    AT_WIFI_STATUS_DISCONNECTED,
    AT_WIFI_STATUS_CONNECTING,
    AT_WIFI_STATUS_GOT_IP,
    AT_WIFI_STATUS_CONNECTION_FAILED,
} at_wifi_status_t;

// WiFi connection info
typedef struct {
    char ssid[33];
    char bssid[18];
    int8_t rssi;
    uint8_t channel;
    at_wifi_status_t status;
} at_wifi_info_t;

// IP information
typedef struct {
    char ip[16];
    char gateway[16];
    char netmask[16];
    char mac[18];
} at_ip_info_t;

/**
 * @brief Set WiFi mode
 * @param mode WiFi mode
 * @return AT response code
 */
at_response_t at_wifi_set_mode(at_wifi_mode_t mode);

/**
 * @brief Get WiFi mode
 * @param mode Output: current mode
 * @return AT response code
 */
at_response_t at_wifi_get_mode(at_wifi_mode_t *mode);

/**
 * @brief Connect to WiFi AP
 * @param ssid SSID
 * @param password Password (NULL for open network)
 * @param timeout_ms Timeout in milliseconds
 * @return AT response code
 */
at_response_t at_wifi_connect(const char *ssid, const char *password, uint32_t timeout_ms);

/**
 * @brief Disconnect from WiFi AP
 * @return AT response code
 */
at_response_t at_wifi_disconnect(void);

/**
 * @brief Check if connected to WiFi
 * @return true if connected and has IP
 */
bool at_wifi_is_connected(void);

/**
 * @brief Get WiFi connection info
 * @param info Output: connection info
 * @return AT response code
 */
at_response_t at_wifi_get_info(at_wifi_info_t *info);

/**
 * @brief Get IP information
 * @param info Output: IP info
 * @return AT response code
 */
at_response_t at_wifi_get_ip(at_ip_info_t *info);

/**
 * @brief Scan for WiFi networks
 * @param ssid_filter Optional SSID filter (NULL for all)
 * @param results Buffer for results string
 * @param results_len Buffer length
 * @return AT response code
 */
at_response_t at_wifi_scan(const char *ssid_filter, char *results, size_t results_len);

/**
 * @brief Enable auto-connect on boot
 * @param enable true to enable
 * @return AT response code
 */
at_response_t at_wifi_set_auto_connect(bool enable);

/**
 * @brief Set hostname
 * @param hostname Hostname string
 * @return AT response code
 */
at_response_t at_wifi_set_hostname(const char *hostname);

/**
 * @brief Configure DHCP
 * @param enable true to enable DHCP
 * @return AT response code
 */
at_response_t at_wifi_set_dhcp(bool enable);

/**
 * @brief Set static IP (disables DHCP)
 * @param ip IP address
 * @param gateway Gateway
 * @param netmask Netmask
 * @return AT response code
 */
at_response_t at_wifi_set_static_ip(const char *ip, const char *gateway, const char *netmask);

/**
 * @brief Get connection status
 * @return WiFi status
 */
at_wifi_status_t at_wifi_get_status(void);

/**
 * @brief Quick initialization: set mode + connect
 * @param ssid SSID
 * @param password Password
 * @return AT response code
 */
at_response_t at_wifi_init_and_connect(const char *ssid, const char *password);

#ifdef __cplusplus
}
#endif
