#include "coproc_uart.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG_UART = "coproc_uart";
static TaskHandle_t s_rx_task = NULL;
static bool s_uart_inited = false;
static uart_port_t s_active_port = -1;
static SemaphoreHandle_t s_c6_ready_sem = NULL;
static bool s_c6_ready_flag = false;

typedef struct {
    uart_port_t port;
    int tx_pin;
    int rx_pin;
    const char *name; // "u1" or "u2"
    TaskHandle_t rx_task;
    bool inited;
} uart_candidate_t;

// Note: UART2 (GPIO35/36) connects to C6 UART0 (GPIO17/16)
static uart_candidate_t s_candidates[] = {
    { UART_NUM_2, 35, 36, "u2", NULL, false },  // P4 GPIO35(TX)→C6 GPIO17(RX), P4 GPIO36(RX)→C6 GPIO16(TX)
    { UART_NUM_1, 45, 46, "u1", NULL, false },
    { UART_NUM_0, -1, -1, "u0", NULL, false },
};

static void coproc_uart_rx_task(void *arg)
{
#if CONFIG_COPROC_UART_ENABLE
    const uart_candidate_t *cand = (const uart_candidate_t *)arg;
    const uart_port_t port = cand ? cand->port : CONFIG_COPROC_UART_NUM;
    uint8_t ch;
    char line[256];
    size_t n = 0;
    int heartbeat = 0;
    int total_bytes = 0;

    ESP_LOGI(TAG_UART, "UART RX task started on port %d (%s)", port, cand ? cand->name : "main");

    for (;;) {
        int r = uart_read_bytes(port, &ch, 1, pdMS_TO_TICKS(100));

        // Heartbeat a cada 10 segundos
        if (++heartbeat >= 100) {  // 100 * 100ms = 10s
            ESP_LOGI(TAG_UART, "UART%d RX alive (total_bytes=%d, buf=%d)", port, total_bytes, n);
            heartbeat = 0;
        }

        if (r == 1) {
            total_bytes++;
            if (ch == '\n' || ch == '\r') {
                if (n > 0) {
                    line[n] = 0;
                    // Log all received lines
                    ESP_LOGI(TAG_UART, "C6[%s]: %s", cand ? cand->name : "cfg", line);

                    // Detect C6_READY signal
                    if (!s_c6_ready_flag && strstr(line, "C6_READY")) {
                        s_c6_ready_flag = true;
                        if (s_c6_ready_sem) {
                            xSemaphoreGive(s_c6_ready_sem);
                        }
                        ESP_LOGI(TAG_UART, "🟢 C6_READY detected! C6 is ready for commands");
                    }

                    // Process C6 acknowledgments
                    if (strstr(line, "C6_ACK") || strstr(line, "C6:")) {
                        ESP_LOGI(TAG_UART, "✓ C6 acknowledged: %s", line);
                    }

                    // Set active port on first message
                    if (s_active_port < 0 && cand) {
                        s_active_port = port;
                        ESP_LOGI(TAG_UART, "Active UART set to %s (port %d)", cand->name, port);
                    }

                    // TODO: Add callback mechanism here if needed
                    // For now, all C6 messages are logged with "C6[u2]:" prefix

                    n = 0;
                }
            } else if (n < sizeof(line) - 1) {
                line[n++] = (char)ch;
            } else {
                // overflow, flush
                line[n] = 0;
                ESP_LOGI(TAG_UART, "C6[%s]: %s", cand ? cand->name : "cfg", line);
                n = 0;
            }
        }
    }
#else
    (void)arg;
    vTaskDelete(NULL);
#endif
}

esp_err_t coproc_uart_init(void)
{
#if !CONFIG_COPROC_UART_ENABLE
    return ESP_ERR_INVALID_STATE;
#else
    if (CONFIG_COPROC_UART_TX_PIN < 0) {
        ESP_LOGW(TAG_UART, "COPROC_UART_TX_PIN < 0; skipping init");
        return ESP_ERR_INVALID_ARG;
    }
    const uart_port_t port = CONFIG_COPROC_UART_NUM;
    const int tx_pin = CONFIG_COPROC_UART_TX_PIN;
    const int rx_pin = CONFIG_COPROC_UART_RX_PIN;

    // CRITICAL: Follow ESP-IDF examples - install driver FIRST
    // Order: 1) uart_driver_install, 2) uart_param_config, 3) uart_set_pin

    if (!uart_is_driver_installed(port)) {
        ESP_LOGI(TAG_UART, "Installing UART%d driver with RX=4096, TX=2048", port);
        ESP_ERROR_CHECK(uart_driver_install(port, 4096, 2048, 0, NULL, 0));
    } else {
        ESP_LOGI(TAG_UART, "UART%d driver already installed", port);
    }

    uart_config_t cfg = {
        .baud_rate = CONFIG_COPROC_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG_UART, "Configuring UART%d with baud=%d", port, CONFIG_COPROC_UART_BAUD);
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));

    // CRITICAL: Explicitly set pins (don't rely on defaults)
    ESP_LOGI(TAG_UART, "Setting UART%d pins: TX=%d, RX=%d", port, tx_pin, rx_pin);
    ESP_ERROR_CHECK(uart_set_pin(port, tx_pin,
                                 (rx_pin >= 0 ? rx_pin : UART_PIN_NO_CHANGE),
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_uart_inited = true;

    // Create semaphore for C6_READY detection
    if (!s_c6_ready_sem) {
        s_c6_ready_sem = xSemaphoreCreateBinary();
        if (!s_c6_ready_sem) {
            ESP_LOGE(TAG_UART, "Failed to create C6_READY semaphore");
        }
    }

    ESP_LOGI(TAG_UART, "UART%d ready: TX=GPIO%d, RX=GPIO%d, baud=%d",
             port, tx_pin, rx_pin, CONFIG_COPROC_UART_BAUD);
    return ESP_OK;
#endif
}

esp_err_t coproc_uart_send_line(const char *line)
{
#if !CONFIG_COPROC_UART_ENABLE
    return ESP_ERR_INVALID_STATE;
#else
    if (!line) return ESP_ERR_INVALID_ARG;
    // Forçar uso do UART configurado para comunicação com C6
    uart_port_t port = CONFIG_COPROC_UART_NUM;

    // Verificar se o driver UART está instalado antes de tentar enviar
    if (!uart_is_driver_installed(port)) {
        ESP_LOGW(TAG_UART, "UART%d driver not installed, skipping write", port);
        return ESP_ERR_INVALID_STATE;
    }

    // CRITICAL FIX: Flush RX buffer before sending to prevent interference
    size_t rx_bytes = 0;
    uart_get_buffered_data_len(port, &rx_bytes);
    if (rx_bytes > 0) {
        // Log at INFO level to debug communication issues
        ESP_LOGI(TAG_UART, "Note: Flushing %d bytes from RX buffer before send", (int)rx_bytes);
        uart_flush_input(port);
    }

    // CRITICAL FIX: Clear TX FIFO before writing to ensure clean transmission
    uart_wait_tx_done(port, pdMS_TO_TICKS(100));

    // Compose message with newline
    char buf[256];
    size_t n = strnlen(line, sizeof(buf) - 2);
    memcpy(buf, line, n);
    buf[n++] = '\n';

    // blocking write
    ESP_LOGI(TAG_UART, "TX→C6 [UART%d]: %.*s", port, (int)(n-1), buf);
    int w = uart_write_bytes(port, buf, n);

    if (w == (int)n) {
        // Wait for TX to complete (ensures bytes are actually transmitted)
        esp_err_t wait_ret = uart_wait_tx_done(port, pdMS_TO_TICKS(1000));
        if (wait_ret != ESP_OK) {
            ESP_LOGW(TAG_UART, "TX wait_done timeout: %s", esp_err_to_name(wait_ret));
        }
        return ESP_OK;
    }
    ESP_LOGW(TAG_UART, "UART%d write incomplete: %d/%d bytes", port, w, (int)n);
    int ok = 0;
    for (unsigned i = 0; i < sizeof(s_candidates)/sizeof(s_candidates[0]); ++i) {
        if (s_candidates[i].inited) {
            int w2 = uart_write_bytes(s_candidates[i].port, buf, n);
            ok |= (w2 == (int)n);
        }
    }
    return ok ? ESP_OK : ESP_FAIL;
#endif
}

esp_err_t coproc_uart_start_rx_log(void)
{
#if !CONFIG_COPROC_UART_ENABLE
    return ESP_ERR_INVALID_STATE;
#else
    if (s_rx_task) return ESP_OK;
    BaseType_t ok = xTaskCreatePinnedToCore(
        coproc_uart_rx_task, "coproc_uart_rx", 3072, NULL, 2, &s_rx_task, 0);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
#endif
}

esp_err_t coproc_uart_probe_and_start_rx_log(void)
{
#if !CONFIG_COPROC_UART_ENABLE
    return ESP_ERR_INVALID_STATE;
#else
    const int baud = CONFIG_COPROC_UART_BAUD;
    for (unsigned i = 0; i < sizeof(s_candidates)/sizeof(s_candidates[0]); ++i) {
        uart_candidate_t *c = &s_candidates[i];
        if (c->inited) continue;
        uart_config_t cfg = {
            .baud_rate = baud,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        uart_param_config(c->port, &cfg);
        if (c->tx_pin >= 0 || c->rx_pin >= 0) {
            int tx = (c->tx_pin >= 0) ? c->tx_pin : UART_PIN_NO_CHANGE;
            int rx = (c->rx_pin >= 0) ? c->rx_pin : UART_PIN_NO_CHANGE;
            uart_set_pin(c->port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        } else {
            // Keep default mapping
            uart_set_pin(c->port, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        }
        if (!uart_is_driver_installed(c->port)) {
            uart_driver_install(c->port, 4096, 2048, 0, NULL, 0);
        }
        c->inited = true;
        BaseType_t ok = xTaskCreatePinnedToCore(coproc_uart_rx_task, c->name, 3072, (void*)c, 2, &c->rx_task, 0);
        if (ok != pdPASS) return ESP_ERR_NO_MEM;
        ESP_LOGI(TAG_UART, "Probing %s: UART%d TX=%d RX=%d @%d", c->name, c->port, c->tx_pin, c->rx_pin, baud);
    }
    return ESP_OK;
#endif
}

esp_err_t coproc_uart_force_uart2_log(void)
{
#if !CONFIG_COPROC_UART_ENABLE
    return ESP_ERR_INVALID_STATE;
#else
    uart_candidate_t *c = NULL;
    // find uart2 candidate
    for (unsigned i = 0; i < sizeof(s_candidates)/sizeof(s_candidates[0]); ++i) {
        if (s_candidates[i].port == UART_NUM_2) { c = &s_candidates[i]; break; }
    }
    if (c == NULL) return ESP_ERR_NOT_FOUND;

    const int baud = CONFIG_COPROC_UART_BAUD;
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(c->port, &cfg);
    uart_set_pin(c->port, c->tx_pin, c->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (!uart_is_driver_installed(c->port)) {
        uart_driver_install(c->port, 4096, 2048, 0, NULL, 0);
    }
    c->inited = true;
    s_active_port = c->port;
    if (!c->rx_task) {
        BaseType_t ok = xTaskCreatePinnedToCore(coproc_uart_rx_task, c->name, 3072, (void*)c, 2, &c->rx_task, 0);
        if (ok != pdPASS) return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG_UART, "Forced log on %s: UART%d TX=%d RX=%d @%d", c->name, c->port, c->tx_pin, c->rx_pin, baud);
    return ESP_OK;
#endif
}

esp_err_t coproc_uart_wait_for_c6_ready(uint32_t timeout_ms)
{
#if !CONFIG_COPROC_UART_ENABLE
    return ESP_ERR_INVALID_STATE;
#else
    // If already ready, return immediately
    if (s_c6_ready_flag) {
        ESP_LOGI(TAG_UART, "C6 already ready (fast path)");
        return ESP_OK;
    }

    if (!s_c6_ready_sem) {
        ESP_LOGE(TAG_UART, "C6_READY semaphore not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG_UART, "⏳ Waiting for C6_READY signal (timeout: %lu ms)...", timeout_ms);
    TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);

    if (xSemaphoreTake(s_c6_ready_sem, ticks) == pdTRUE) {
        ESP_LOGI(TAG_UART, "✅ C6_READY received! C6 is fully initialized");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG_UART, "⏰ Timeout waiting for C6_READY after %lu ms", timeout_ms);
        return ESP_ERR_TIMEOUT;
    }
#endif
}
