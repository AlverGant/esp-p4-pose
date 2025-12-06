/**
 * @file at_wifi.c
 * @brief WiFi management via ESP-AT commands implementation
 */

#include "at_wifi.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "at_wifi";

// Cache connection status
static at_wifi_status_t s_wifi_status = AT_WIFI_STATUS_UNKNOWN;

at_response_t at_wifi_set_mode(at_wifi_mode_t mode)
{
    ESP_LOGI(TAG, "Setting WiFi mode to %d", mode);
    return at_send_cmd_fmt(NULL, 0, 2000, "+CWMODE=%d", mode);
}

at_response_t at_wifi_get_mode(at_wifi_mode_t *mode)
{
    char response[64];
    at_response_t ret = at_send_cmd("+CWMODE?", response, sizeof(response), 2000);

    if (ret == AT_OK && mode) {
        // Parse "+CWMODE:1" format
        char *p = strstr(response, "+CWMODE:");
        if (p) {
            *mode = (at_wifi_mode_t)atoi(p + 8);
        }
    }

    return ret;
}

at_response_t at_wifi_connect(const char *ssid, const char *password, uint32_t timeout_ms)
{
    if (!ssid) {
        return AT_ERROR;
    }

    s_wifi_status = AT_WIFI_STATUS_CONNECTING;

    ESP_LOGI(TAG, "Connecting to WiFi: %s", ssid);

    at_response_t ret;
    if (password && password[0]) {
        ret = at_send_cmd_fmt(NULL, 0, timeout_ms, "+CWJAP=\"%s\",\"%s\"", ssid, password);
    } else {
        ret = at_send_cmd_fmt(NULL, 0, timeout_ms, "+CWJAP=\"%s\",\"\"", ssid);
    }

    if (ret == AT_OK) {
        s_wifi_status = AT_WIFI_STATUS_CONNECTED;
        ESP_LOGI(TAG, "WiFi connected to %s", ssid);
    } else if (ret == AT_ALREADY_CONNECTED) {
        s_wifi_status = AT_WIFI_STATUS_CONNECTED;
        ESP_LOGI(TAG, "Already connected to %s", ssid);
        ret = AT_OK;
    } else {
        s_wifi_status = AT_WIFI_STATUS_CONNECTION_FAILED;
        ESP_LOGE(TAG, "Failed to connect to %s: %s", ssid, at_response_to_str(ret));
    }

    return ret;
}

at_response_t at_wifi_disconnect(void)
{
    ESP_LOGI(TAG, "Disconnecting from WiFi");
    at_response_t ret = at_send_cmd("+CWQAP", NULL, 0, 5000);

    if (ret == AT_OK) {
        s_wifi_status = AT_WIFI_STATUS_DISCONNECTED;
    }

    return ret;
}

bool at_wifi_is_connected(void)
{
    char response[256];
    at_response_t ret = at_send_cmd("+CWJAP?", response, sizeof(response), 2000);

    if (ret == AT_OK) {
        // If connected, response contains "+CWJAP:ssid,bssid,..."
        // If not connected, response might be "No AP" or similar
        if (strstr(response, "+CWJAP:\"") != NULL) {
            s_wifi_status = AT_WIFI_STATUS_CONNECTED;
            return true;
        }
    }

    s_wifi_status = AT_WIFI_STATUS_DISCONNECTED;
    return false;
}

at_response_t at_wifi_get_info(at_wifi_info_t *info)
{
    if (!info) {
        return AT_ERROR;
    }

    memset(info, 0, sizeof(*info));

    char response[256];
    at_response_t ret = at_send_cmd("+CWJAP?", response, sizeof(response), 2000);

    if (ret == AT_OK) {
        // Parse "+CWJAP:"ssid","bssid",channel,rssi"
        char *p = strstr(response, "+CWJAP:\"");
        if (p) {
            p += 8;  // Skip +CWJAP:"
            char *end = strchr(p, '"');
            if (end) {
                size_t len = end - p;
                if (len < sizeof(info->ssid)) {
                    memcpy(info->ssid, p, len);
                    info->ssid[len] = '\0';
                }

                // Parse BSSID
                p = end + 2;  // Skip ","
                if (*p == '"') {
                    p++;
                    end = strchr(p, '"');
                    if (end) {
                        len = end - p;
                        if (len < sizeof(info->bssid)) {
                            memcpy(info->bssid, p, len);
                            info->bssid[len] = '\0';
                        }

                        // Parse channel and RSSI
                        p = end + 2;  // Skip ,"
                        info->channel = atoi(p);
                        p = strchr(p, ',');
                        if (p) {
                            info->rssi = atoi(p + 1);
                        }
                    }
                }
            }
            info->status = AT_WIFI_STATUS_CONNECTED;
        } else {
            info->status = AT_WIFI_STATUS_DISCONNECTED;
        }
    }

    return ret;
}

at_response_t at_wifi_get_ip(at_ip_info_t *info)
{
    if (!info) {
        return AT_ERROR;
    }

    memset(info, 0, sizeof(*info));

    char response[256];
    at_response_t ret = at_send_cmd("+CIFSR", response, sizeof(response), 2000);

    if (ret == AT_OK) {
        // Parse +CIFSR:STAIP,"x.x.x.x"
        char *p = strstr(response, "STAIP,\"");
        if (p) {
            p += 7;
            char *end = strchr(p, '"');
            if (end && (end - p) < 16) {
                memcpy(info->ip, p, end - p);
            }
        }

        // Parse MAC
        p = strstr(response, "STAMAC,\"");
        if (p) {
            p += 8;
            char *end = strchr(p, '"');
            if (end && (end - p) < 18) {
                memcpy(info->mac, p, end - p);
            }
        }

        // Get gateway from CIPSTA
        at_response_t ret2 = at_send_cmd("+CIPSTA?", response, sizeof(response), 2000);
        if (ret2 == AT_OK) {
            p = strstr(response, "gateway:\"");
            if (p) {
                p += 9;
                char *end = strchr(p, '"');
                if (end && (end - p) < 16) {
                    memcpy(info->gateway, p, end - p);
                }
            }
            p = strstr(response, "netmask:\"");
            if (p) {
                p += 9;
                char *end = strchr(p, '"');
                if (end && (end - p) < 16) {
                    memcpy(info->netmask, p, end - p);
                }
            }
        }

        if (info->ip[0]) {
            s_wifi_status = AT_WIFI_STATUS_GOT_IP;
            ESP_LOGI(TAG, "IP: %s, MAC: %s", info->ip, info->mac);
        }
    }

    return ret;
}

at_response_t at_wifi_scan(const char *ssid_filter, char *results, size_t results_len)
{
    ESP_LOGI(TAG, "Scanning for WiFi networks...");

    if (ssid_filter && ssid_filter[0]) {
        return at_send_cmd_fmt(results, results_len, 10000, "+CWLAP=\"%s\"", ssid_filter);
    } else {
        return at_send_cmd("+CWLAP", results, results_len, 10000);
    }
}

at_response_t at_wifi_set_auto_connect(bool enable)
{
    ESP_LOGI(TAG, "Setting auto-connect: %s", enable ? "enabled" : "disabled");
    return at_send_cmd_fmt(NULL, 0, 2000, "+CWAUTOCONN=%d", enable ? 1 : 0);
}

at_response_t at_wifi_set_hostname(const char *hostname)
{
    if (!hostname) {
        return AT_ERROR;
    }

    ESP_LOGI(TAG, "Setting hostname: %s", hostname);
    return at_send_cmd_fmt(NULL, 0, 2000, "+CWHOSTNAME=\"%s\"", hostname);
}

at_response_t at_wifi_set_dhcp(bool enable)
{
    // Mode 1 = Station, bit 0 = enable DHCP
    ESP_LOGI(TAG, "Setting DHCP: %s", enable ? "enabled" : "disabled");
    return at_send_cmd_fmt(NULL, 0, 2000, "+CWDHCP=1,%d", enable ? 1 : 0);
}

at_response_t at_wifi_set_static_ip(const char *ip, const char *gateway, const char *netmask)
{
    if (!ip) {
        return AT_ERROR;
    }

    // First disable DHCP
    at_wifi_set_dhcp(false);

    ESP_LOGI(TAG, "Setting static IP: %s", ip);

    if (gateway && netmask) {
        return at_send_cmd_fmt(NULL, 0, 2000, "+CIPSTA=\"%s\",\"%s\",\"%s\"",
                               ip, gateway, netmask);
    } else {
        return at_send_cmd_fmt(NULL, 0, 2000, "+CIPSTA=\"%s\"", ip);
    }
}

at_wifi_status_t at_wifi_get_status(void)
{
    // Refresh status
    at_wifi_is_connected();
    return s_wifi_status;
}

at_response_t at_wifi_init_and_connect(const char *ssid, const char *password)
{
    at_response_t ret;

    // Disable echo for cleaner parsing
    at_set_echo(false);

    // Set station mode
    ret = at_wifi_set_mode(AT_WIFI_MODE_STA);
    if (ret != AT_OK && ret != AT_NO_CHANGE) {
        ESP_LOGE(TAG, "Failed to set WiFi mode");
        return ret;
    }

    // Enable auto-connect
    at_wifi_set_auto_connect(true);

    // Connect to AP
    ret = at_wifi_connect(ssid, password, 30000);
    if (ret != AT_OK) {
        return ret;
    }

    // Verify we got an IP
    at_ip_info_t ip_info;
    ret = at_wifi_get_ip(&ip_info);
    if (ret == AT_OK && ip_info.ip[0]) {
        ESP_LOGI(TAG, "WiFi initialized. IP: %s", ip_info.ip);
    }

    return ret;
}
