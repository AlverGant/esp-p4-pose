/**
 * @file at_client.h
 * @brief ESP-AT Command Client for ESP32-P4 to ESP32-C6 communication
 *
 * This module provides a complete AT command interface for controlling
 * the ESP32-C6 running ESP-AT firmware via UART.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// AT Response codes
typedef enum {
    AT_OK = 0,
    AT_ERROR,
    AT_FAIL,
    AT_BUSY,
    AT_TIMEOUT,
    AT_SEND_OK,
    AT_SEND_FAIL,
    AT_ALREADY_CONNECTED,
    AT_NO_CHANGE,
    AT_UNKNOWN,
} at_response_t;

// AT client state
typedef enum {
    AT_STATE_IDLE = 0,
    AT_STATE_WAITING_RESPONSE,
    AT_STATE_WAITING_DATA,
    AT_STATE_WAITING_PROMPT,
} at_state_t;

// Callback for unsolicited result codes (URCs)
typedef void (*at_urc_callback_t)(const char *urc, void *user_data);

// Callback for data reception (e.g., +IPD, +MQTTSUBRECV)
typedef void (*at_data_callback_t)(const char *topic, const uint8_t *data, size_t len, void *user_data);

// AT client configuration
typedef struct {
    uint32_t default_timeout_ms;    // Default command timeout (default: 5000)
    uint32_t connect_timeout_ms;    // WiFi/MQTT connect timeout (default: 30000)
    at_urc_callback_t urc_callback; // Callback for URCs
    at_data_callback_t data_callback; // Callback for received data
    void *user_data;                // User data for callbacks
} at_client_config_t;

#define AT_CLIENT_CONFIG_DEFAULT() { \
    .default_timeout_ms = 5000, \
    .connect_timeout_ms = 30000, \
    .urc_callback = NULL, \
    .data_callback = NULL, \
    .user_data = NULL, \
}

/**
 * @brief Initialize the AT client
 * @param config Client configuration (NULL for defaults)
 * @return ESP_OK on success
 */
esp_err_t at_client_init(const at_client_config_t *config);

/**
 * @brief Deinitialize the AT client
 */
void at_client_deinit(void);

/**
 * @brief Check if AT client is initialized
 */
bool at_client_is_initialized(void);

/**
 * @brief Send an AT command and wait for response
 * @param cmd Command string (without "AT" prefix and CR/LF)
 * @param response Buffer for response (can be NULL)
 * @param response_len Max response buffer length
 * @param timeout_ms Timeout in milliseconds (0 for default)
 * @return AT response code
 */
at_response_t at_send_cmd(const char *cmd, char *response, size_t response_len, uint32_t timeout_ms);

/**
 * @brief Send AT command with formatted arguments
 */
at_response_t at_send_cmd_fmt(char *response, size_t response_len, uint32_t timeout_ms, const char *fmt, ...);

/**
 * @brief Send raw data (for commands like AT+CIPSEND)
 * @param data Data buffer
 * @param len Data length
 * @param timeout_ms Timeout
 * @return AT response code
 */
at_response_t at_send_data(const uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Send command that expects '>' prompt, then send data
 *
 * Used for commands like AT+HTTPCPOST that require binary data input.
 * Flow: Send cmd -> Wait for '>' -> Send data -> Wait for OK/ERROR
 *
 * @param cmd Command string (without "AT" prefix)
 * @param data Binary data to send after prompt
 * @param data_len Data length
 * @param response Buffer for response (can be NULL)
 * @param response_len Max response buffer length
 * @param prompt_timeout_ms Timeout waiting for '>' prompt
 * @param response_timeout_ms Timeout waiting for final response
 * @return AT response code
 */
at_response_t at_send_cmd_with_data(const char *cmd,
                                     const uint8_t *data, size_t data_len,
                                     char *response, size_t response_len,
                                     uint32_t prompt_timeout_ms,
                                     uint32_t response_timeout_ms);

/**
 * @brief Test AT communication (sends "AT" and expects "OK")
 * @return true if communication is working
 */
bool at_test(void);

/**
 * @brief Reset the ESP-AT module
 * @return AT response code
 */
at_response_t at_reset(void);

/**
 * @brief Get ESP-AT version info
 * @param version Buffer for version string
 * @param len Buffer length
 * @return AT response code
 */
at_response_t at_get_version(char *version, size_t len);

/**
 * @brief Enable/disable AT echo
 * @param enable true to enable echo
 * @return AT response code
 */
at_response_t at_set_echo(bool enable);

/**
 * @brief Process incoming UART data (call from RX task)
 * @param data Received byte
 */
void at_client_process_byte(uint8_t data);

/**
 * @brief Process a complete line (alternative to byte-by-byte)
 * @param line Null-terminated line
 */
void at_client_process_line(const char *line);

/**
 * @brief Get current AT client state
 */
at_state_t at_client_get_state(void);

/**
 * @brief Convert AT response to string
 */
const char *at_response_to_str(at_response_t resp);

#ifdef __cplusplus
}
#endif
