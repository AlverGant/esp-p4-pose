#include <stdio.h>
#include <string.h>
#include <stdbool.h>

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
#include "driver/uart.h"
#include "esp_sntp.h"
#include "esp_crt_bundle.h"
#include "coproc_sdio_slave.h"
#include "hosted_alert_server.h"

static const char *TAG = "c6_msg";
static const int C6_UART_PORT = UART_NUM_0;

static void uart_tx_line(const char *s)
{
    if (!s) return;
    char buf[256];
    size_t n = strnlen(s, sizeof(buf) - 2);
    memcpy(buf, s, n);
    buf[n++] = '\n';
    uart_write_bytes(C6_UART_PORT, buf, n);
}

// Forward declaration
esp_err_t telegram_send(const char *text);

// Combined function that sends via SDIO and UART
static void send_response(const char *s)
{
    // Try SDIO first
    esp_err_t ret = coproc_sdio_slave_send_line(s);
    if (ret != ESP_OK) {
        // Fallback to UART
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
    ESP_LOGI(TAG, "%s FALL: %s", transport, message);

    if (feedback) {
        char ack[64];
        snprintf(ack, sizeof(ack), "C6: FALL recebido via %s", transport);
        send_response(ack);
        send_response(message);
    }

    int64_t now = esp_timer_get_time();
    if (s_cooldown > 0) {
        int64_t last;
        portENTER_CRITICAL(&s_fall_spin);
        last = s_last_sent;
        portEXIT_CRITICAL(&s_fall_spin);

        int64_t elapsed = now - last;
        if (elapsed < s_cooldown) {
            ESP_LOGI(TAG, "%s cooldown ativo (%lld ms restantes)", transport,
                     (long long)((s_cooldown - elapsed) / 1000));
            if (feedback) {
                send_response("C6: Cooldown, ignorado");
            }
            return ESP_ERR_INVALID_STATE;
        }
    }

    esp_err_t err = telegram_send(message);
    if (err == ESP_OK) {
        portENTER_CRITICAL(&s_fall_spin);
        s_last_sent = esp_timer_get_time();
        portEXIT_CRITICAL(&s_fall_spin);
    } else if (feedback) {
        char line[64];
        snprintf(line, sizeof(line), "C6: Telegram erro %d", (int)err);
        send_response(line);
    }
    return err;
}

static esp_err_t handle_incoming_message(const char *message, const char *transport, bool feedback)
{
    if (!message || !*message) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "%s RX: %s", transport, message);

    if (feedback) {
        char ack[64];
        snprintf(ack, sizeof(ack), "C6: %s msg recebida", transport);
        send_response(ack);
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
            ESP_LOGI(TAG, "Wi-Fi ready");
            send_response("C6: Wi-Fi conectado");
            return ESP_OK;
        }
    }
    ESP_LOGW(TAG, "Wi-Fi IP not acquired yet");
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
            ESP_LOGI(TAG, "SNTP ok: %04d-%02d-%02d %02d:%02d", t.tm_year+1900, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min);
            send_response("C6: SNTP OK");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGW(TAG, "SNTP timeout");
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
        ESP_LOGI(TAG, "Telegram status=%d", sc);
        char l[64]; snprintf(l, sizeof l, "C6: Telegram status=%d", sc); send_response(l);
        if (sc != 200) err = ESP_FAIL;
    } else {
        ESP_LOGW(TAG, "HTTP error %s", esp_err_to_name(err));
        char l[64]; snprintf(l, sizeof l, "C6: HTTP erro %d", (int)err); send_response(l);
    }
    esp_http_client_cleanup(c);
    return err;
}

// Telegram photo send function
esp_err_t telegram_send_photo(const uint8_t *image_data, size_t image_len, const char *caption)
{
    if (!image_data || image_len == 0) return ESP_ERR_INVALID_ARG;
    if (strlen(CONFIG_TELEGRAM_BOT_TOKEN) == 0 || strlen(CONFIG_TELEGRAM_CHAT_ID) == 0) {
        ESP_LOGW(TAG, "Telegram token/chat_id not configured");
        return ESP_ERR_INVALID_STATE;
    }

    char url[256];
    snprintf(url, sizeof(url), "https://api.telegram.org/bot%s/sendPhoto", CONFIG_TELEGRAM_BOT_TOKEN);

    // For now, log that we would send a photo
    // Full implementation would require multipart/form-data encoding
    ESP_LOGI(TAG, "Would send photo to Telegram: %zu bytes, caption: %s", image_len, caption ? caption : "none");
    send_response("C6: Photo upload not yet implemented");

    // TODO: Implement multipart/form-data upload
    // This requires more complex HTTP handling with boundaries

    return ESP_ERR_NOT_SUPPORTED;
}

static void uart_reader_task(void *arg)
{
    const int uart_port = C6_UART_PORT; // U0TXD/U0RXD
    ESP_LOGI(TAG, "=== UART READER TASK STARTED ===");
    // Driver and params are set in app_main(). Just ensure RX buffer exists.
    if (!uart_is_driver_installed(uart_port)) {
        (void)uart_driver_install(uart_port, 1024, 0, 0, NULL, 0);
    }

    char line[256]; size_t n = 0;
    uint8_t ch;
    ESP_LOGI(TAG, "UART reader: waiting for messages...");
    for (;;) {
        int r = uart_read_bytes(uart_port, &ch, 1, pdMS_TO_TICKS(100));
        if (r == 1) {
            if (ch == '\n' || ch == '\r') {
                line[n] = 0;
                if (n > 0 && line[0]) {
                    // Log every line temporarily for debugging
                    if (line[0] == 'F') {
                        ESP_LOGI(TAG, "UART DEBUG: Got line starting with 'F': '%s' (len=%d)", line, n);
                    }
                    // Check for FALL message first
                    if (strncmp(line, "FALL", 4) == 0) {
                        (void)handle_incoming_message(line, "UART", true);
                    }
                    // Don't log ESP-IDF log lines to avoid feedback loop
                }
                n = 0;  // Reset buffer after processing line
            } else if (n < sizeof(line)-1) {
                line[n++] = (char)ch;
            } else {
                // overflow, reset
                n = 0;
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== C6 APP_MAIN START - FIRMWARE VERSION 2025-09-30-SDIO-FIRST ===");
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "=== NVS INIT OK ===");

    // Initialize cooldown from config
    s_cooldown = (int64_t)CONFIG_TELEGRAM_COOLDOWN_SEC * 1000000LL;

    // Ensure UART0 is ready before any uart_tx_line() - keep as backup
    {
        const int uart_port = C6_UART_PORT;
        uart_config_t uc = {
            .baud_rate = CONFIG_UART_BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        uart_param_config(uart_port, &uc);
        // Use default UART0 pins (connected to P4 via hardware)
        // Leave pins as default - hardware routes to P4 GPIO35/36
        uart_set_pin(uart_port, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        ESP_LOGI(TAG, "C6: UART0 configured with default pins (connected to P4)");
        if (!uart_is_driver_installed(uart_port)) {
            uart_driver_install(uart_port, 1024, 0, 0, NULL, 0);
        }
        ESP_LOGI(TAG, "=== UART INIT OK ===");
    }

    // Initialize SDIO slave BEFORE Wi-Fi - must be ready when P4 connects
    ESP_LOGI(TAG, "=== INITIALIZING SDIO SLAVE ===");
    esp_err_t sdio_ret = coproc_sdio_slave_init();
    if (sdio_ret == ESP_OK) {
        ESP_LOGI(TAG, "SDIO slave initialized successfully");
        uart_tx_line("C6: SDIO slave OK - aguardando P4");
        // Start SDIO RX with message handler
        coproc_sdio_slave_start_rx(sdio_message_handler);
    } else {
        ESP_LOGW(TAG, "SDIO slave init failed: %s, using UART only", esp_err_to_name(sdio_ret));
        uart_tx_line("C6: SDIO falhou - usando UART apenas");
    }

    // Start Wi-Fi after SDIO is ready
    ESP_LOGI(TAG, "=== STARTING WIFI ===");
    wifi_connect();
    sntp_sync();

#if CONFIG_HOSTED_ALERT_SERVER_ENABLE
    {
        esp_err_t http_err = hosted_alert_server_start(hosted_alert_callback);
        if (http_err == ESP_OK) {
            ESP_LOGI(TAG, "Hosted alert HTTP server iniciado na porta %d", CONFIG_HOSTED_ALERT_SERVER_PORT);
            send_response("C6: Servidor HTTP pronto para alertas");
        } else {
            ESP_LOGW(TAG, "Falha ao iniciar servidor HTTP: %s", esp_err_to_name(http_err));
        }
    }
#endif

    if (sdio_ret == ESP_OK) {
        send_response("C6: Sistema iniciado com SDIO+UART");
    }

    // Start UART reader as backup (will also handle UART-based messages)
    xTaskCreate(uart_reader_task, "uart_reader", 4096, NULL, 5, NULL);
}
