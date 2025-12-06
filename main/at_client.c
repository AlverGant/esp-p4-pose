/**
 * @file at_client.c
 * @brief ESP-AT Command Client Implementation
 */

#include "at_client.h"
#include "coproc_uart.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static const char *TAG = "at_client";

// Response buffer
#define AT_RX_BUF_SIZE 2048
#define AT_LINE_BUF_SIZE 512

// Static state
static struct {
    bool initialized;
    at_client_config_t config;
    at_state_t state;
    SemaphoreHandle_t response_sem;
    SemaphoreHandle_t mutex;

    // Response accumulation
    char line_buf[AT_LINE_BUF_SIZE];
    size_t line_pos;
    char response_buf[AT_RX_BUF_SIZE];
    size_t response_pos;
    at_response_t last_response;
    bool response_complete;

    // Data mode
    size_t expected_data_len;
    size_t received_data_len;
} s_at = {0};

// Forward declarations
static at_response_t parse_response_code(const char *line);
static void handle_urc(const char *line);

esp_err_t at_client_init(const at_client_config_t *config)
{
    if (s_at.initialized) {
        ESP_LOGW(TAG, "AT client already initialized");
        return ESP_OK;
    }

    // Apply configuration
    if (config) {
        s_at.config = *config;
    } else {
        at_client_config_t default_config = AT_CLIENT_CONFIG_DEFAULT();
        s_at.config = default_config;
    }

    // Create semaphores
    s_at.response_sem = xSemaphoreCreateBinary();
    if (!s_at.response_sem) {
        ESP_LOGE(TAG, "Failed to create response semaphore");
        return ESP_ERR_NO_MEM;
    }

    s_at.mutex = xSemaphoreCreateMutex();
    if (!s_at.mutex) {
        vSemaphoreDelete(s_at.response_sem);
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    s_at.state = AT_STATE_IDLE;
    s_at.initialized = true;

    ESP_LOGI(TAG, "AT client initialized (timeout=%lums)", s_at.config.default_timeout_ms);
    return ESP_OK;
}

void at_client_deinit(void)
{
    if (!s_at.initialized) return;

    if (s_at.response_sem) {
        vSemaphoreDelete(s_at.response_sem);
        s_at.response_sem = NULL;
    }
    if (s_at.mutex) {
        vSemaphoreDelete(s_at.mutex);
        s_at.mutex = NULL;
    }

    s_at.initialized = false;
    ESP_LOGI(TAG, "AT client deinitialized");
}

bool at_client_is_initialized(void)
{
    return s_at.initialized;
}

at_state_t at_client_get_state(void)
{
    return s_at.state;
}

const char *at_response_to_str(at_response_t resp)
{
    switch (resp) {
        case AT_OK: return "OK";
        case AT_ERROR: return "ERROR";
        case AT_FAIL: return "FAIL";
        case AT_BUSY: return "BUSY";
        case AT_TIMEOUT: return "TIMEOUT";
        case AT_SEND_OK: return "SEND OK";
        case AT_SEND_FAIL: return "SEND FAIL";
        case AT_ALREADY_CONNECTED: return "ALREADY CONNECTED";
        case AT_NO_CHANGE: return "NO CHANGE";
        default: return "UNKNOWN";
    }
}

static at_response_t parse_response_code(const char *line)
{
    if (strcmp(line, "OK") == 0) return AT_OK;
    if (strcmp(line, "ERROR") == 0) return AT_ERROR;
    if (strcmp(line, "FAIL") == 0) return AT_FAIL;
    if (strstr(line, "busy") != NULL) return AT_BUSY;
    if (strcmp(line, "SEND OK") == 0) return AT_SEND_OK;
    if (strcmp(line, "SEND FAIL") == 0) return AT_SEND_FAIL;
    if (strstr(line, "ALREADY CONNECTED") != NULL) return AT_ALREADY_CONNECTED;
    if (strstr(line, "no change") != NULL) return AT_NO_CHANGE;
    return AT_UNKNOWN;
}

static void handle_urc(const char *line)
{
    ESP_LOGD(TAG, "URC: %s", line);

    // Common URCs
    if (strncmp(line, "+CWJAP:", 7) == 0) {
        ESP_LOGI(TAG, "WiFi event: %s", line);
    } else if (strncmp(line, "+MQTTCONNECTED", 14) == 0) {
        ESP_LOGI(TAG, "MQTT connected");
    } else if (strncmp(line, "+MQTTDISCONNECTED", 17) == 0) {
        ESP_LOGW(TAG, "MQTT disconnected");
    } else if (strncmp(line, "+MQTTSUBRECV:", 13) == 0) {
        ESP_LOGI(TAG, "MQTT message: %s", line);
        // TODO: Parse and call data_callback
    } else if (strncmp(line, "+IPD", 4) == 0) {
        ESP_LOGD(TAG, "Data received: %s", line);
    } else if (strncmp(line, "WIFI ", 5) == 0) {
        ESP_LOGI(TAG, "WiFi status: %s", line);
    }

    // Call user callback if registered
    if (s_at.config.urc_callback) {
        s_at.config.urc_callback(line, s_at.config.user_data);
    }
}

void at_client_process_line(const char *line)
{
    if (!s_at.initialized || !line || !line[0]) return;

    // VERBOSE: Log all received lines for debugging
    ESP_LOGI(TAG, "RX[state=%d]: \"%s\"", s_at.state, line);

    // Check for response codes
    at_response_t resp = parse_response_code(line);
    ESP_LOGI(TAG, "  -> parsed: %s (state=%d)", at_response_to_str(resp), s_at.state);

    if (resp != AT_UNKNOWN) {
        // This is a final response
        ESP_LOGI(TAG, "  -> FINAL RESPONSE: %s, waiting=%d", at_response_to_str(resp),
                 s_at.state == AT_STATE_WAITING_RESPONSE);
        if (s_at.state == AT_STATE_WAITING_RESPONSE) {
            s_at.last_response = resp;
            s_at.response_complete = true;
            s_at.state = AT_STATE_IDLE;
            ESP_LOGI(TAG, "  -> Signaling semaphore!");
            xSemaphoreGive(s_at.response_sem);
        }
    } else if (line[0] == '+' || strncmp(line, "WIFI", 4) == 0 ||
               strncmp(line, "MQTT", 4) == 0 || strncmp(line, "ready", 5) == 0) {
        // Unsolicited Result Code or status
        if (s_at.state == AT_STATE_WAITING_RESPONSE) {
            // Append to response buffer
            size_t len = strlen(line);
            if (s_at.response_pos + len + 2 < AT_RX_BUF_SIZE) {
                memcpy(s_at.response_buf + s_at.response_pos, line, len);
                s_at.response_pos += len;
                s_at.response_buf[s_at.response_pos++] = '\n';
                s_at.response_buf[s_at.response_pos] = '\0';
            }
        }
        handle_urc(line);
    } else if (line[0] == '>') {
        // Data prompt
        if (s_at.state == AT_STATE_WAITING_PROMPT) {
            s_at.state = AT_STATE_WAITING_DATA;
            xSemaphoreGive(s_at.response_sem);
        }
    } else if (s_at.state == AT_STATE_WAITING_RESPONSE) {
        // Append to response buffer (intermediate data)
        size_t len = strlen(line);
        if (s_at.response_pos + len + 2 < AT_RX_BUF_SIZE) {
            memcpy(s_at.response_buf + s_at.response_pos, line, len);
            s_at.response_pos += len;
            s_at.response_buf[s_at.response_pos++] = '\n';
            s_at.response_buf[s_at.response_pos] = '\0';
        }
    }
}

void at_client_process_byte(uint8_t data)
{
    if (!s_at.initialized) return;

    // Immediate detection of '>' prompt (doesn't require newline)
    if (data == '>' && s_at.state == AT_STATE_WAITING_PROMPT) {
        ESP_LOGI(TAG, "Got '>' prompt!");
        s_at.state = AT_STATE_WAITING_DATA;
        xSemaphoreGive(s_at.response_sem);
        return;
    }

    if (data == '\n' || data == '\r') {
        if (s_at.line_pos > 0) {
            s_at.line_buf[s_at.line_pos] = '\0';
            at_client_process_line(s_at.line_buf);
            s_at.line_pos = 0;
        }
    } else if (s_at.line_pos < AT_LINE_BUF_SIZE - 1) {
        s_at.line_buf[s_at.line_pos++] = (char)data;
    }
}

at_response_t at_send_cmd(const char *cmd, char *response, size_t response_len, uint32_t timeout_ms)
{
    if (!s_at.initialized) {
        ESP_LOGE(TAG, "AT client not initialized");
        return AT_ERROR;
    }

    if (!cmd) {
        return AT_ERROR;
    }

    // Take mutex
    if (xSemaphoreTake(s_at.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex");
        return AT_BUSY;
    }

    // Use default timeout if not specified
    if (timeout_ms == 0) {
        timeout_ms = s_at.config.default_timeout_ms;
    }

    // Prepare for response
    s_at.response_buf[0] = '\0';
    s_at.response_pos = 0;
    s_at.response_complete = false;
    s_at.last_response = AT_UNKNOWN;
    s_at.state = AT_STATE_WAITING_RESPONSE;

    // Clear any pending semaphore
    xSemaphoreTake(s_at.response_sem, 0);

    // Build and send command
    char at_cmd[512];
    if (cmd[0] == '\0') {
        // Just "AT"
        snprintf(at_cmd, sizeof(at_cmd), "AT");
    } else if (strncmp(cmd, "AT", 2) == 0) {
        // Already has AT prefix
        snprintf(at_cmd, sizeof(at_cmd), "%s", cmd);
    } else {
        // Add AT prefix
        snprintf(at_cmd, sizeof(at_cmd), "AT%s", cmd);
    }

    ESP_LOGI(TAG, "TX: %s", at_cmd);
    esp_err_t err = coproc_uart_send_line(at_cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(err));
        s_at.state = AT_STATE_IDLE;
        xSemaphoreGive(s_at.mutex);
        return AT_ERROR;
    }

    // Wait for response
    if (xSemaphoreTake(s_at.response_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "Command timeout: %s", at_cmd);
        s_at.state = AT_STATE_IDLE;
        xSemaphoreGive(s_at.mutex);
        return AT_TIMEOUT;
    }

    // Copy response if buffer provided
    if (response && response_len > 0) {
        strncpy(response, s_at.response_buf, response_len - 1);
        response[response_len - 1] = '\0';
    }

    at_response_t result = s_at.last_response;
    ESP_LOGI(TAG, "Response: %s", at_response_to_str(result));

    xSemaphoreGive(s_at.mutex);
    return result;
}

at_response_t at_send_cmd_fmt(char *response, size_t response_len, uint32_t timeout_ms, const char *fmt, ...)
{
    char cmd[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(cmd, sizeof(cmd), fmt, args);
    va_end(args);
    return at_send_cmd(cmd, response, response_len, timeout_ms);
}

at_response_t at_send_data(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (!s_at.initialized || !data || len == 0) {
        return AT_ERROR;
    }

    // Wait for prompt first
    s_at.state = AT_STATE_WAITING_PROMPT;
    if (xSemaphoreTake(s_at.response_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        s_at.state = AT_STATE_IDLE;
        return AT_TIMEOUT;
    }

    // Send raw data
    uart_port_t port = CONFIG_COPROC_UART_NUM;
    int written = uart_write_bytes(port, data, len);
    if (written != (int)len) {
        ESP_LOGE(TAG, "Failed to write data: %d/%d", written, (int)len);
        s_at.state = AT_STATE_IDLE;
        return AT_ERROR;
    }

    uart_wait_tx_done(port, pdMS_TO_TICKS(1000));

    // Wait for SEND OK/FAIL
    s_at.state = AT_STATE_WAITING_RESPONSE;
    if (xSemaphoreTake(s_at.response_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        s_at.state = AT_STATE_IDLE;
        return AT_TIMEOUT;
    }

    s_at.state = AT_STATE_IDLE;
    return s_at.last_response;
}

at_response_t at_send_cmd_with_data(const char *cmd,
                                     const uint8_t *data, size_t data_len,
                                     char *response, size_t response_len,
                                     uint32_t prompt_timeout_ms,
                                     uint32_t response_timeout_ms)
{
    if (!s_at.initialized) {
        ESP_LOGE(TAG, "AT client not initialized");
        return AT_ERROR;
    }

    if (!cmd || !data || data_len == 0) {
        return AT_ERROR;
    }

    // Take mutex
    if (xSemaphoreTake(s_at.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire mutex");
        return AT_BUSY;
    }

    // Use defaults if not specified
    if (prompt_timeout_ms == 0) prompt_timeout_ms = 5000;
    if (response_timeout_ms == 0) response_timeout_ms = 30000;

    // Prepare for prompt
    s_at.response_buf[0] = '\0';
    s_at.response_pos = 0;
    s_at.response_complete = false;
    s_at.last_response = AT_UNKNOWN;
    s_at.state = AT_STATE_WAITING_PROMPT;  // Wait for '>'

    // Clear any pending semaphore
    xSemaphoreTake(s_at.response_sem, 0);

    // Build and send command
    char at_cmd[1024];
    if (strncmp(cmd, "AT", 2) == 0) {
        snprintf(at_cmd, sizeof(at_cmd), "%s", cmd);
    } else {
        snprintf(at_cmd, sizeof(at_cmd), "AT%s", cmd);
    }

    ESP_LOGI(TAG, "TX (data cmd): %s", at_cmd);
    esp_err_t err = coproc_uart_send_line(at_cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(err));
        s_at.state = AT_STATE_IDLE;
        xSemaphoreGive(s_at.mutex);
        return AT_ERROR;
    }

    // Wait for '>' prompt
    ESP_LOGI(TAG, "Waiting for '>' prompt (timeout=%lu ms)...", prompt_timeout_ms);
    if (xSemaphoreTake(s_at.response_sem, pdMS_TO_TICKS(prompt_timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout waiting for '>' prompt");
        s_at.state = AT_STATE_IDLE;
        xSemaphoreGive(s_at.mutex);
        return AT_TIMEOUT;
    }

    ESP_LOGI(TAG, "Got prompt, sending %d bytes of data...", (int)data_len);

    // Send raw data in chunks
    uart_port_t port = CONFIG_COPROC_UART_NUM;
    const size_t chunk_size = 1024;
    size_t sent = 0;

    while (sent < data_len) {
        size_t to_send = (data_len - sent > chunk_size) ? chunk_size : (data_len - sent);
        int written = uart_write_bytes(port, data + sent, to_send);
        if (written < 0) {
            ESP_LOGE(TAG, "UART write failed");
            s_at.state = AT_STATE_IDLE;
            xSemaphoreGive(s_at.mutex);
            return AT_ERROR;
        }
        sent += written;

        // Progress logging and yield
        if (sent % 10240 == 0) {
            ESP_LOGI(TAG, "Data TX progress: %d/%d bytes (%.1f%%)",
                     (int)sent, (int)data_len, 100.0f * sent / data_len);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // Wait for TX to complete
    uart_wait_tx_done(port, pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Data sent (%d bytes), waiting for response...", (int)sent);

    // Now wait for final response (OK/ERROR)
    s_at.state = AT_STATE_WAITING_RESPONSE;
    xSemaphoreTake(s_at.response_sem, 0);  // Clear any pending

    if (xSemaphoreTake(s_at.response_sem, pdMS_TO_TICKS(response_timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout waiting for response after data transfer");
        s_at.state = AT_STATE_IDLE;
        xSemaphoreGive(s_at.mutex);
        return AT_TIMEOUT;
    }

    // Copy response if buffer provided
    if (response && response_len > 0) {
        strncpy(response, s_at.response_buf, response_len - 1);
        response[response_len - 1] = '\0';
    }

    at_response_t result = s_at.last_response;
    ESP_LOGI(TAG, "Data command response: %s", at_response_to_str(result));

    s_at.state = AT_STATE_IDLE;
    xSemaphoreGive(s_at.mutex);
    return result;
}

bool at_test(void)
{
    at_response_t resp = at_send_cmd("", NULL, 0, 2000);
    return (resp == AT_OK);
}

at_response_t at_reset(void)
{
    return at_send_cmd("+RST", NULL, 0, 5000);
}

at_response_t at_get_version(char *version, size_t len)
{
    return at_send_cmd("+GMR", version, len, 2000);
}

at_response_t at_set_echo(bool enable)
{
    char cmd[8];
    snprintf(cmd, sizeof(cmd), "E%d", enable ? 1 : 0);
    return at_send_cmd(cmd, NULL, 0, 1000);
}
