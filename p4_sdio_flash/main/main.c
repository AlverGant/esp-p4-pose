/* P4-EYE -> C6 SDIO flashing (using esp-serial-flasher as component) */

#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp32_sdio_port.h"
#include "esp_loader.h"
#include "example_common.h"
#include "freertos/FreeRTOS.h"

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

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    // C6_U0TXD -> P4 GPIO36 (RX), C6_U0RXD -> P4 GPIO35 (TX)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPIO_NUM_35, GPIO_NUM_36,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BUF_LEN * 4, BUF_LEN * 4, 0, NULL, 0));

    while (1) {
        int rx = uart_read_bytes(UART_NUM_2, buf, BUF_LEN - 1, 100 / portTICK_PERIOD_MS);
        if (rx > 0) { buf[rx] = '\0'; printf("%s", buf); }
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
        xTaskCreate(slave_monitor, "slave_monitor", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
    }
    vTaskDelete(NULL);
}
