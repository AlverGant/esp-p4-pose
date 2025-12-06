/**
 * @file at_ota.c
 * @brief OTA firmware update via ESP-AT commands implementation
 */

#include "at_ota.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "at_ota";

// Callbacks
static at_ota_progress_cb_t s_progress_cb = NULL;
static at_ota_result_cb_t s_result_cb = NULL;
static void *s_progress_user_data = NULL;
static void *s_result_user_data = NULL;

void at_ota_set_progress_callback(at_ota_progress_cb_t callback, void *user_data)
{
    s_progress_cb = callback;
    s_progress_user_data = user_data;
}

void at_ota_set_result_callback(at_ota_result_cb_t callback, void *user_data)
{
    s_result_cb = callback;
    s_result_user_data = user_data;
}

at_response_t at_ota_update_from_cloud(void)
{
    ESP_LOGI(TAG, "Starting OTA update from Espressif cloud...");

    // AT+CIUPDATE downloads firmware from Espressif's OTA server
    // This can take a long time (minutes)
    at_response_t ret = at_send_cmd("+CIUPDATE", NULL, 0, 180000);  // 3 minute timeout

    if (ret == AT_OK) {
        ESP_LOGI(TAG, "OTA update successful! Module will restart.");
        if (s_result_cb) {
            s_result_cb(true, "OTA update successful", s_result_user_data);
        }
    } else {
        ESP_LOGE(TAG, "OTA update failed: %s", at_response_to_str(ret));
        if (s_result_cb) {
            s_result_cb(false, at_response_to_str(ret), s_result_user_data);
        }
    }

    return ret;
}

at_response_t at_ota_update_from_url(const char *url, uint32_t timeout_ms)
{
    if (!url) {
        return AT_ERROR;
    }

    ESP_LOGI(TAG, "Starting OTA update from: %s", url);

    if (timeout_ms == 0) {
        timeout_ms = 180000;  // 3 minutes default
    }

    // AT+CIUPDATE=<ota_type>,<"url">
    // ota_type: 0=all, 1=user1.bin, 2=user2.bin
    at_response_t ret = at_send_cmd_fmt(NULL, 0, timeout_ms,
        "+CIUPDATE=0,\"%s\"", url);

    if (ret == AT_OK) {
        ESP_LOGI(TAG, "OTA update from URL successful!");
        if (s_result_cb) {
            s_result_cb(true, "OTA update successful", s_result_user_data);
        }
    } else {
        ESP_LOGE(TAG, "OTA update from URL failed: %s", at_response_to_str(ret));
        if (s_result_cb) {
            s_result_cb(false, at_response_to_str(ret), s_result_user_data);
        }
    }

    return ret;
}

at_response_t at_ota_check_update(char *version_out, size_t version_len)
{
    ESP_LOGI(TAG, "Checking for OTA updates...");

    char response[256];
    at_response_t ret = at_send_cmd("+CIUPDATECHECK", response, sizeof(response), 30000);

    if (ret == AT_OK && version_out && version_len > 0) {
        // Parse version from response if available
        char *p = strstr(response, "+CIUPDATECHECK:");
        if (p) {
            // Format: +CIUPDATECHECK:<version>
            p += 15;
            char *end = strchr(p, '\n');
            if (end) {
                size_t len = end - p;
                if (len >= version_len) len = version_len - 1;
                memcpy(version_out, p, len);
                version_out[len] = '\0';
            }
        }
    }

    return ret;
}

at_response_t at_ota_get_partition_info(char *response, size_t response_len)
{
    ESP_LOGI(TAG, "Getting partition info...");

    // AT+SYSFLASH? returns flash partition information
    return at_send_cmd("+SYSFLASH?", response, response_len, 5000);
}

at_response_t at_ota_rollback(void)
{
    ESP_LOGW(TAG, "Rolling back to previous firmware...");

    // AT+SYSROLLBACK rolls back to the previous valid firmware
    at_response_t ret = at_send_cmd("+SYSROLLBACK", NULL, 0, 10000);

    if (ret == AT_OK) {
        ESP_LOGI(TAG, "Rollback initiated. Module will restart.");
    } else {
        ESP_LOGE(TAG, "Rollback failed: %s", at_response_to_str(ret));
    }

    return ret;
}

at_response_t at_ota_restart(void)
{
    ESP_LOGI(TAG, "Restarting ESP-AT module...");
    return at_send_cmd("+RST", NULL, 0, 5000);
}

at_response_t at_ota_restore_factory(void)
{
    ESP_LOGW(TAG, "Restoring factory settings...");

    // AT+RESTORE resets all parameters to factory defaults
    at_response_t ret = at_send_cmd("+RESTORE", NULL, 0, 10000);

    if (ret == AT_OK) {
        ESP_LOGI(TAG, "Factory restore complete. Module will restart.");
    } else {
        ESP_LOGE(TAG, "Factory restore failed: %s", at_response_to_str(ret));
    }

    return ret;
}
