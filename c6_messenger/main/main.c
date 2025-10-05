#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_client.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_sntp.h"
#include "esp_crt_bundle.h"
#include "coproc_sdio_slave.h"
#include "hosted_alert_server.h"

// Debug LED DISABLED - testing if GPIO8 conflicts with UART
// #define DEBUG_LED_GPIO 8

static const char *TAG = "c6_msg";
static const int C6_UART_PORT = UART_NUM_0;

static void uart_tx_line(const char *s)
{
    if (!s) return;
    char buf[256];
    size_t n = strnlen(s, sizeof(buf) - 2);
    memcpy(buf, s, n);
    buf[n++] = '\n';

    int written = uart_write_bytes(C6_UART_PORT, buf, n);
    if (written > 0) {
        uart_wait_tx_done(C6_UART_PORT, pdMS_TO_TICKS(1000));
    }
}

// Forward declaration
esp_err_t telegram_send(const char *text);

// Combined function that sends via SDIO and UART
static void send_response(const char *s)
{
    // Try SDIO first (only works with main P4 app, not p4_sdio_flash)
    esp_err_t ret = coproc_sdio_slave_send_line(s);
    if (ret != ESP_OK) {
        // Fallback to UART if SDIO fails or not available
        uart_tx_line(s);
    }
}

// Forward declarations
// No heartbeat/test spam; keep only FALL handling

// Global variables for message handling
static int64_t s_last_sent = 0;
static int64_t s_cooldown = 0;
static portMUX_TYPE s_fall_spin = portMUX_INITIALIZER_UNLOCKED;

static esp_err_t handle_fall_alert(const char *message, const char *transport, bool feedback);
static esp_err_t handle_incoming_message(const char *message, const char *transport, bool feedback);

#if CONFIG_HOSTED_ALERT_SERVER_ENABLE
static hosted_alert_result_t hosted_alert_callback(const char *message, size_t len)
{
    (void)len;
    if (!message || !*message) {
        return HOSTED_ALERT_RESULT_IGNORED;
    }

    esp_err_t err = handle_incoming_message(message, "HTTP", false);
    if (err == ESP_OK) {
        return HOSTED_ALERT_RESULT_OK;
    }
    if (err == ESP_ERR_INVALID_STATE) {
        return HOSTED_ALERT_RESULT_COOLDOWN;
    }
    if (err == ESP_ERR_INVALID_ARG) {
        return HOSTED_ALERT_RESULT_IGNORED;
    }
    return HOSTED_ALERT_RESULT_ERROR;
}
#endif

// SDIO message handler
static void sdio_message_handler(const char *message)
{
    if (!message || !*message) {
        return;
    }
    (void)handle_incoming_message(message, "SDIO", true);
}

static esp_err_t handle_fall_alert(const char *message, const char *transport, bool feedback)
{
    (void)transport;
    (void)feedback;

    // CRITICAL: NO LOGGING - called from uart_reader_task which reads from UART0
    // Any ESP_LOGI() here will write to UART0, causing feedback loop and lockup

    // Send to Telegram (no cooldown - removed for testing)
    esp_err_t err = telegram_send(message);

    return err;
}

static esp_err_t handle_incoming_message(const char *message, const char *transport, bool feedback)
{
    // CRITICAL: NO LOGGING - called from uart_reader_task which reads from UART0
    // Any ESP_LOGI() here will write to UART0, causing feedback loop and lockup

    if (!message || !*message) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strncmp(message, "FALL", 4) == 0) {
        return handle_fall_alert(message, transport, feedback);
    }

    return ESP_ERR_INVALID_ARG;
}

static esp_err_t wifi_connect(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t w = {0};
    strncpy((char *)w.sta.ssid, CONFIG_WIFI_SSID, sizeof(w.sta.ssid)-1);
    strncpy((char *)w.sta.password, CONFIG_WIFI_PASSWORD, sizeof(w.sta.password)-1);
    // Se a senha estiver vazia, permita redes abertas
    w.sta.threshold.authmode = (strlen(CONFIG_WIFI_PASSWORD) > 0) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &w));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Tentar conectar explicitamente
    esp_wifi_connect();
    ESP_LOGI(TAG, "Connecting to '%s'", CONFIG_WIFI_SSID);

    // Basic wait for IP
    for (int i = 0; i < 150; ++i) {
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip) == ESP_OK && ip.ip.addr) {
            return ESP_OK;
        }
    }
    return ESP_OK;
}

static void sntp_sync(void)
{
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    // aguarda até o tempo ficar razoável
    for (int i = 0; i < 20; ++i) {
        time_t now = 0; struct tm t = {0};
        time(&now); localtime_r(&now, &t);
        if (t.tm_year > (2016 - 1900)) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t telegram_send(const char *text)
{
    if (!text || !*text) return ESP_ERR_INVALID_ARG;
    if (strlen(CONFIG_TELEGRAM_BOT_TOKEN) == 0 || strlen(CONFIG_TELEGRAM_CHAT_ID) == 0) {
        ESP_LOGW(TAG, "Telegram token/chat_id not configured");
        return ESP_ERR_INVALID_STATE;
    }
    char url[256];
    // Correct Telegram API endpoint: https://api.telegram.org/bot<TOKEN>/sendMessage
    snprintf(url, sizeof(url), "https://api.telegram.org/bot%s/sendMessage", CONFIG_TELEGRAM_BOT_TOKEN);

    // Escape basic chars for JSON
    char safe[256]; size_t j = 0;
    for (const char *p = text; *p && j < sizeof(safe)-1; ++p) {
        if (*p == '"' && j < sizeof(safe)-2) { safe[j++]='\\'; safe[j++]='"'; }
        else if (*p == '\n' && j < sizeof(safe)-2) { safe[j++]='\\'; safe[j++]='n'; }
        else { safe[j++]=*p; }
    }
    safe[j]=0;

    char body[384];
    snprintf(body, sizeof(body), "{\"chat_id\":\"%s\",\"text\":\"%s\"}", CONFIG_TELEGRAM_CHAT_ID, safe);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) return ESP_FAIL;
    esp_http_client_set_header(c, "Content-Type", "application/json");
    esp_http_client_set_post_field(c, body, strlen(body));
    esp_err_t err = esp_http_client_perform(c);
    if (err == ESP_OK) {
        int sc = esp_http_client_get_status_code(c);
        if (sc != 200) err = ESP_FAIL;
    }
    esp_http_client_cleanup(c);
    return err;
}

// HTTP event handler to capture error responses
static char s_http_response_buffer[512];
static int s_http_response_len = 0;

static esp_err_t telegram_http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            // Capture response data
            if (s_http_response_len + evt->data_len < sizeof(s_http_response_buffer)) {
                memcpy(s_http_response_buffer + s_http_response_len, evt->data, evt->data_len);
                s_http_response_len += evt->data_len;
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

// Telegram photo send function - multipart/form-data upload
esp_err_t telegram_send_photo(const uint8_t *image_data, size_t image_len, const char *caption)
{
    if (!image_data || image_len == 0) return ESP_ERR_INVALID_ARG;
    if (strlen(CONFIG_TELEGRAM_BOT_TOKEN) == 0 || strlen(CONFIG_TELEGRAM_CHAT_ID) == 0) {
        ESP_LOGW(TAG, "Telegram token/chat_id not configured");
        return ESP_ERR_INVALID_STATE;
    }

    char url[256];
    snprintf(url, sizeof(url), "https://api.telegram.org/bot%s/sendPhoto", CONFIG_TELEGRAM_BOT_TOKEN);

    // Multipart/form-data boundary
    const char *boundary = "----FormBoundary7MA4YWxkTrZu0gW";

    // Build multipart body
    char *body = NULL;
    size_t body_len = 0;

    // Calculate body size
    size_t header_len = 512;  // Estimate for headers
    body_len = header_len + image_len + 512;  // Headers + image + footer

    // C6 doesn't have SPIRAM, use internal RAM
    body = malloc(body_len);
    if (!body) {
        ESP_LOGE(TAG, "Failed to allocate multipart body");
        return ESP_ERR_NO_MEM;
    }

    // Build multipart form-data body
    size_t offset = 0;

    // Part 1: chat_id field
    offset += snprintf(body + offset, body_len - offset,
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n"
        "%s\r\n",
        boundary, CONFIG_TELEGRAM_CHAT_ID);

    // Part 2: photo file
    offset += snprintf(body + offset, body_len - offset,
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"photo\"; filename=\"fall.jpg\"\r\n"
        "Content-Type: image/jpeg\r\n\r\n",
        boundary);

    // Copy image data
    memcpy(body + offset, image_data, image_len);
    offset += image_len;

    // Must add CRLF after binary data before next boundary
    offset += snprintf(body + offset, body_len - offset, "\r\n");

    // Part 3: caption (optional)
    if (caption && strlen(caption) > 0) {
        offset += snprintf(body + offset, body_len - offset,
            "--%s\r\n"
            "Content-Disposition: form-data; name=\"caption\"\r\n\r\n"
            "%s\r\n",
            boundary, caption);
    }

    // Final boundary
    offset += snprintf(body + offset, body_len - offset, "--%s--\r\n", boundary);

    ESP_LOGI(TAG, "Multipart body size: %d bytes (image: %d bytes)", (int)offset, (int)image_len);

    // Reset response buffer
    s_http_response_len = 0;
    memset(s_http_response_buffer, 0, sizeof(s_http_response_buffer));

    // Configure HTTP client with event handler
    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 30000,  // 30s timeout for photo upload
        .event_handler = telegram_http_event_handler,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        free(body);
        return ESP_FAIL;
    }

    // Set multipart/form-data content type with boundary
    char content_type[128];
    snprintf(content_type, sizeof(content_type), "multipart/form-data; boundary=%s", boundary);
    esp_http_client_set_header(client, "Content-Type", content_type);
    esp_http_client_set_post_field(client, body, offset);

    // Perform request
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);

        ESP_LOGI(TAG, "Telegram photo upload status: %d, response_len: %d", status_code, s_http_response_len);

        // Log response for debugging
        if (s_http_response_len > 0) {
            s_http_response_buffer[s_http_response_len] = '\0';
            if (status_code != 200) {
                ESP_LOGE(TAG, "Telegram error response: %s", s_http_response_buffer);
            } else {
                ESP_LOGI(TAG, "Telegram response: %s", s_http_response_buffer);
            }
        }

        if (status_code != 200) {
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP POST failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    free(body);

    return err;
}

// REMOVED: heartbeat_task - causes UART spam and interferes with P4→C6 communication

// State machine for UART reader
typedef enum {
    UART_STATE_TEXT,    // Reading text lines
    UART_STATE_BINARY   // Reading binary photo data
} uart_state_t;

static void uart_reader_task(void *arg)
{
    const int uart_port = C6_UART_PORT; // U0TXD/U0RXD

    char line[256];
    size_t n = 0;
    uint8_t ch;
    int msg_count = 0;
    int heartbeat = 0;
    bool led_state = false;

    // State machine
    uart_state_t state = UART_STATE_TEXT;
    size_t photo_size = 0;
    size_t photo_received = 0;
    uint8_t *photo_buf = NULL;

    // CRITICAL: Disable ALL logging in this task to prevent UART interference
    // Logging to UART0 while reading from UART0 creates a feedback loop that causes lockup

    for (;;) {
        int r = uart_read_bytes(uart_port, &ch, 1, pdMS_TO_TICKS(100));

        // Heartbeat: LED DISABLED for debugging
        if (++heartbeat >= 100) {  // 100 * 100ms = 10s
            led_state = !led_state;
            // gpio_set_level(DEBUG_LED_GPIO, led_state);
            heartbeat = 0;
        }

        if (r == 1) {
            if (state == UART_STATE_TEXT) {
                // TEXT MODE: Read lines
                if (ch == '\n' || ch == '\r') {
                    if (n > 0) {
                        line[n] = 0;
                        msg_count++;

                        // Check if it's a PHOTO command
                        if (strncmp(line, "PHOTO:", 6) == 0) {
                            // Parse photo size
                            photo_size = atoi(line + 6);
                            if (photo_size > 0 && photo_size < 100000) {  // Max 100KB (C6 has limited RAM)
                                // Allocate buffer for photo - C6 doesn't have SPIRAM, use internal RAM
                                photo_buf = malloc(photo_size);
                                if (photo_buf) {
                                    photo_received = 0;
                                    state = UART_STATE_BINARY;
                                    uart_tx_line("C6_PHOTO_READY");
                                    // NO LOGGING - prevent UART interference
                                } else {
                                    uart_tx_line("C6_PHOTO_ERR_MEM");
                                }
                            } else {
                                uart_tx_line("C6_PHOTO_ERR_SIZE");
                            }
                        } else {
                            // Normal text command
                            uart_tx_line("C6_ACK");

                            // Process message silently
                            if (strncmp(line, "FALL", 4) == 0) {
                                handle_incoming_message(line, "UART", true);
                            }
                        }
                    }
                    n = 0;
                } else if (n < sizeof(line)-1) {
                    line[n++] = (char)ch;
                } else {
                    // Overflow: reset silently
                    n = 0;
                }
            } else {
                // BINARY MODE: Read photo data
                photo_buf[photo_received++] = ch;

                if (photo_received >= photo_size) {
                    // Photo complete - validate JPEG format
                    bool valid_jpeg = false;
                    if (photo_size >= 4) {
                        // Check JPEG magic bytes: starts with 0xFF 0xD8, ends with 0xFF 0xD9
                        bool valid_start = (photo_buf[0] == 0xFF && photo_buf[1] == 0xD8);
                        bool valid_end = (photo_buf[photo_size-2] == 0xFF && photo_buf[photo_size-1] == 0xD9);
                        valid_jpeg = valid_start && valid_end;

                        ESP_LOGI(TAG, "JPEG validation: start=%d end=%d (first bytes: %02X %02X, last bytes: %02X %02X)",
                                 valid_start, valid_end,
                                 photo_buf[0], photo_buf[1],
                                 photo_buf[photo_size-2], photo_buf[photo_size-1]);
                    }

                    if (!valid_jpeg) {
                        ESP_LOGE(TAG, "Invalid JPEG data received!");
                        uart_tx_line("C6_PHOTO_ERR_INVALID");
                    } else {
                        // Upload to Telegram (WITHOUT caption for now to debug)
                        esp_err_t ret = telegram_send_photo(photo_buf, photo_size, NULL);

                        if (ret == ESP_OK) {
                            uart_tx_line("C6_PHOTO_OK");
                        } else {
                            uart_tx_line("C6_PHOTO_ERR_UPLOAD");
                        }
                    }

                    // Free buffer and return to text mode
                    free(photo_buf);
                    photo_buf = NULL;
                    photo_size = 0;
                    photo_received = 0;
                    state = UART_STATE_TEXT;
                }
            }
        }
    }
}

// Custom vprintf to disable ALL log output to UART
static int null_vprintf(const char *fmt, va_list args) {
    return 0;  // Discard all logs
}

void app_main(void)
{
    // Wait a bit for P4 UART to be ready
    vTaskDelay(pdMS_TO_TICKS(500));

    // Debug LED DISABLED for testing
    // gpio_reset_pin(DEBUG_LED_GPIO);
    // gpio_set_direction(DEBUG_LED_GPIO, GPIO_MODE_OUTPUT);
    // gpio_set_level(DEBUG_LED_GPIO, 0);

    // CRITICAL: Initialize UART0 FIRST before anything else
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install driver first (correct order)
    uart_driver_install(UART_NUM_0, 4096, 2048, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_cfg);
    uart_set_pin(UART_NUM_0, 16, 17, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Send multiple test messages to ensure we see something
    for (int i = 0; i < 5; i++) {
        const char *boot_msg = "C6_BOOT\n";
        uart_write_bytes(UART_NUM_0, boot_msg, 8);
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // NOTE: Keeping logs enabled for debugging (they go to UART0 → P4 monitor)
    // If logs interfere with commands, re-enable null_vprintf
    // esp_log_set_vprintf(null_vprintf);

    ESP_ERROR_CHECK(nvs_flash_init());

    // COOLDOWN DISABLED FOR DEBUGGING
    // s_cooldown = (int64_t)CONFIG_TELEGRAM_COOLDOWN_SEC * 1000000LL;
    // portENTER_CRITICAL(&s_fall_spin);
    // s_last_sent = 0;
    // portEXIT_CRITICAL(&s_fall_spin);

    // UART already initialized at start of app_main()

    // CRITICAL: Start UART reader IMMEDIATELY after UART init
    // This ensures C6 can receive P4 commands ASAP (before WiFi/SNTP delays)
    // INCREASED stack from 4096 to 8192 to prevent overflow during HTTP calls
    xTaskCreate(uart_reader_task, "uart_reader", 8192, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(50)); // Give task time to start

    // SDIO DISABLED: Using UART-only communication with P4
    esp_err_t sdio_ret = ESP_FAIL;  // Force UART-only mode

    // Start Wi-Fi (non-blocking for UART reception)
    ESP_LOGI(TAG, "Connecting to WiFi...");
    wifi_connect();

    ESP_LOGI(TAG, "Syncing time via SNTP...");
    sntp_sync();

    // IMPORTANT: Send C6_READY AFTER WiFi and SNTP are complete
    // This ensures P4 knows C6 is fully ready for Telegram operations
    ESP_LOGI(TAG, "✅ C6 fully initialized - sending C6_READY to P4");
    uart_tx_line("C6_READY");

    // C6 is now ready - will process FALL commands from P4 via UART
    // and forward them to Telegram

#if CONFIG_HOSTED_ALERT_SERVER_ENABLE
    hosted_alert_server_start(hosted_alert_callback);
#endif
}
