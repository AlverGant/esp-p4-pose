/**
 * @file at_ota.h
 * @brief OTA firmware update via ESP-AT commands
 *
 * Supports updating the ESP32-C6 (ESP-AT) firmware over-the-air
 */

#pragma once

#include "at_client.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// OTA progress callback
typedef void (*at_ota_progress_cb_t)(uint32_t received, uint32_t total, void *user_data);

// OTA result callback
typedef void (*at_ota_result_cb_t)(bool success, const char *message, void *user_data);

/**
 * @brief Start OTA update from Espressif cloud
 *
 * Uses AT+CIUPDATE to download firmware from Espressif's OTA server.
 * This updates the ESP-AT firmware on the C6.
 *
 * @return AT response code
 */
at_response_t at_ota_update_from_cloud(void);

/**
 * @brief Start OTA update from custom HTTP server
 *
 * Downloads and installs firmware from specified URL.
 *
 * @param url HTTP/HTTPS URL to firmware binary
 * @param timeout_ms Timeout (0 for default 60s)
 * @return AT response code
 */
at_response_t at_ota_update_from_url(const char *url, uint32_t timeout_ms);

/**
 * @brief Check for available OTA update
 * @param version_out Buffer for available version string
 * @param version_len Buffer length
 * @return AT_OK if update available, AT_NO_CHANGE if up-to-date
 */
at_response_t at_ota_check_update(char *version_out, size_t version_len);

/**
 * @brief Set OTA progress callback
 * @param callback Progress callback
 * @param user_data User data
 */
void at_ota_set_progress_callback(at_ota_progress_cb_t callback, void *user_data);

/**
 * @brief Set OTA result callback
 * @param callback Result callback
 * @param user_data User data
 */
void at_ota_set_result_callback(at_ota_result_cb_t callback, void *user_data);

/**
 * @brief Get OTA partition info
 * @param response Buffer for response
 * @param response_len Buffer length
 * @return AT response code
 */
at_response_t at_ota_get_partition_info(char *response, size_t response_len);

/**
 * @brief Rollback to previous firmware version
 * @return AT response code
 */
at_response_t at_ota_rollback(void);

/**
 * @brief Restart the ESP-AT module
 * @return AT response code (module will restart)
 */
at_response_t at_ota_restart(void);

/**
 * @brief Restore factory settings
 * @return AT response code
 */
at_response_t at_ota_restore_factory(void);

#ifdef __cplusplus
}
#endif
