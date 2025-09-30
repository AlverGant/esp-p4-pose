#include "coproc_uart.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG_UART = "coproc_uart";
static TaskHandle_t s_rx_task = NULL;
static bool s_uart_inited = false;
static uart_port_t s_active_port = -1;

typedef struct {
    uart_port_t port;
    int tx_pin;
    int rx_pin;
    const char *name; // "u1" or "u2"
    TaskHandle_t rx_task;
    bool inited;
} uart_candidate_t;

// Note: add UART0 as a third candidate with NO_CHANGE pins (use default mux)
static uart_candidate_t s_candidates[] = {
    { UART_NUM_1, 35, 36, "u1", NULL, false },
    { UART_NUM_2, 45, 46, "u2", NULL, false },
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
    for (;;) {
        int r = uart_read_bytes(port, &ch, 1, pdMS_TO_TICKS(100));
        if (r == 1) {
            if (ch == '\n' || ch == '\r') {
                if (n > 0) {
                    line[n] = 0;
                    ESP_LOGI(TAG_UART, "C6[%s]: %s", cand ? cand->name : "cfg", line);
                    if (s_active_port < 0 && cand) {
                        s_active_port = port;
                        ESP_LOGI(TAG_UART, "Active UART set to %s (port %d)", cand->name, port);
                    }
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

    // Always ensure params/pins match the configured values even if a driver is already installed
    if (uart_is_driver_installed(port)) {
        ESP_LOGI(TAG_UART, "UART%d driver already installed, reconfiguring pins/params", port);
    }

    uart_config_t cfg = {
        .baud_rate = CONFIG_COPROC_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, tx_pin,
                                 (rx_pin >= 0 ? rx_pin : UART_PIN_NO_CHANGE),
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // RX buffer for C6 logs; TX buffer 0 (blocking write)
    if (!uart_is_driver_installed(port)) {
        ESP_ERROR_CHECK(uart_driver_install(port, 1024, 0, 0, NULL, 0));
    }
    s_uart_inited = true;
    ESP_LOGI(TAG_UART, "UART%d ready TX=%d RX=%d @%d", port, tx_pin, rx_pin, CONFIG_COPROC_UART_BAUD);
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

    // Compose with newline
    char buf[256];
    size_t n = strnlen(line, sizeof(buf) - 2);
    memcpy(buf, line, n);
    buf[n++] = '\n';
    // blocking write
    ESP_LOGI(TAG_UART, "Sending %d bytes to UART%d: '%.*s'", (int)n, port, (int)(n-1), buf);
    int w = uart_write_bytes(port, buf, n);
    ESP_LOGI(TAG_UART, "UART%d write result: %d/%d bytes", port, w, (int)n);
    if (w == (int)n) return ESP_OK;
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
            uart_driver_install(c->port, 1024, 0, 0, NULL, 0);
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
        uart_driver_install(c->port, 1024, 0, 0, NULL, 0);
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
