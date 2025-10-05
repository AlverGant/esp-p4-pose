/* P4-EYE -> C6 SDIO flashing (using esp-serial-flasher as component) */

#include <string.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp32_sdio_port.h"
#include "esp_loader.h"
#include "example_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "p4_sdio_flash";

#define BUF_LEN 128
static uint8_t buf[BUF_LEN] = {0};

void slave_monitor(void *arg)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .source_clk = UART_SCLK_DEFAULT,
#endif
    };

    // CRITICAL: Correct initialization order (follow ESP-IDF examples)
    // 1) uart_driver_install, 2) uart_param_config, 3) uart_set_pin
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BUF_LEN * 4, BUF_LEN * 4, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    // C6_U0TXD -> P4 GPIO36 (RX), C6_U0RXD -> P4 GPIO35 (TX)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPIO_NUM_35, GPIO_NUM_36,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART2 monitor ready (GPIO35=TX, GPIO36=RX)");

    bool c6_ready = false;
    char line[128];
    size_t n = 0;
    int wait_cycles = 0;
    int msg_count = 0;

    while (1) {
        int rx = uart_read_bytes(UART_NUM_2, buf, 1, 100 / portTICK_PERIOD_MS);
        if (rx == 1) {
            char ch = buf[0];
            if (ch == '\n' || ch == '\r') {
                if (n > 0) {
                    line[n] = '\0';
                    printf("%s\n", line);  // Print complete line

                    // Detect C6_READY and start waiting
                    if (!c6_ready && strstr(line, "C6_READY")) {
                        ESP_LOGI(TAG, "✓ C6 is ready! Waiting 30s for WiFi/Telegram...");
                        c6_ready = true;
                        wait_cycles = 0;
                    }
                    n = 0;
                }
            } else if (n < sizeof(line) - 1) {
                line[n++] = ch;
            } else {
                n = 0;  // overflow
            }
        }

        // After C6_READY detected, send test message repeatedly (every 10s after initial 30s)
        if (c6_ready) {
            wait_cycles++;
            // First message at 30s, then every 10s (100 cycles * 100ms = 10s)
            if ((wait_cycles == 300) || (wait_cycles > 300 && (wait_cycles % 100) == 0)) {
                msg_count++;
                ESP_LOGI(TAG, "🧪 Sending FALL test #%d to C6 (testing cooldown behavior)...", msg_count);
                const char *test_msg = "FALL P4→C6→Telegram repeated test\n";
                uart_write_bytes(UART_NUM_2, test_msg, strlen(test_msg));
                uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(1000));
                ESP_LOGI(TAG, "📤 Test #%d sent (t=%ds). Observing C6 response and cooldown...",
                         msg_count, wait_cycles / 10);
            }
        }
    }
}

void app_main(void)
{
    example_binaries_t bin;

    const loader_esp32_sdio_config_t config = {
        .slot = SDMMC_HOST_SLOT_1,
        .max_freq_khz = SDMMC_FREQ_DEFAULT,
        .reset_trigger_pin = GPIO_NUM_9,   // C6_EN
        .boot_pin = GPIO_NUM_33,           // C6_BOOT (IO0)
        .sdio_d0_pin = GPIO_NUM_29,        // SD2_D0
        .sdio_d1_pin = GPIO_NUM_30,        // SD2_D1
        .sdio_d2_pin = GPIO_NUM_31,        // SD2_D2
        .sdio_d3_pin = GPIO_NUM_32,        // SD2_D3
        .sdio_clk_pin = GPIO_NUM_28,       // SD2_CLK
        .sdio_cmd_pin = GPIO_NUM_27,       // SD2_CMD
    };

    if (loader_port_esp32_sdio_init(&config) != ESP_LOADER_SUCCESS) {
        ESP_LOGE(TAG, "SDIO init failed");
        abort();
    }

    if (connect_to_target(0) == ESP_LOADER_SUCCESS) {
        get_example_binaries(esp_loader_get_target(), &bin);

        // Erase entire flash to ensure clean state
        ESP_LOGI(TAG, "Erasing entire flash (this may take a while)...");
        esp_loader_error_t erase_err = esp_loader_flash_start(0, 0x200000, sizeof(uint8_t));
        if (erase_err == ESP_LOADER_SUCCESS) {
            ESP_LOGI(TAG, "Flash erased successfully");
        } else {
            ESP_LOGW(TAG, "Flash erase failed: %d, continuing anyway", erase_err);
        }

        ESP_LOGI(TAG, "Loading bootloader...");
        flash_binary(bin.boot.data, bin.boot.size, bin.boot.addr);
        ESP_LOGI(TAG, "Loading partition table...");
        flash_binary(bin.part.data, bin.part.size, bin.part.addr);
        ESP_LOGI(TAG, "Loading app...");
        flash_binary(bin.app.data,  bin.app.size,  bin.app.addr);
        ESP_LOGI(TAG, "Done! Resetting target...");
        esp_loader_reset_target();

        // Force boot pin (GPIO33/C6_BOOT/IO0) HIGH to boot from flash (not download mode)
        ESP_LOGI(TAG, "Ensuring C6 boots from flash (not download mode)...");
        gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_NUM_33, 1);  // HIGH = boot from flash
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Trigger another reset to ensure C6 boots with boot pin HIGH
        gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_NUM_9, 0);  // Reset LOW
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_9, 1);  // Reset HIGH (release)

        vTaskDelay(500 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "***** Target (C6) logs *****");
        // CRITICAL FIX: Increase stack size to prevent overflow (was 2048, now 8192)
        xTaskCreate(slave_monitor, "slave_monitor", 8192, NULL, configMAX_PRIORITIES - 1, NULL);
    }
    vTaskDelete(NULL);
}
