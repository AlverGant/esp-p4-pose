/* P4-EYE -> C6 UART flashing - ESP-AT Firmware (Multiple Binaries)
 *
 * This flashes the ESP-AT firmware to the ESP32-C6 on the P4-Eye board.
 * Uses UART interface (more reliable than SDIO for large binaries).
 * ESP-AT requires 6 separate binaries at specific addresses.
 */

#include <string.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp32_port.h"
#include "esp_loader.h"
#include "example_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "p4_uart_flash_at";

// Higher baudrate for faster flashing (will fall back to 115200 for monitoring)
#define HIGHER_BAUDRATE 460800

#define BUF_LEN 128
static uint8_t buf[BUF_LEN] = {0};

void slave_monitor(void *arg)
{
    // Reset to 115200 for monitoring AT firmware output
    uart_flush_input(UART_NUM_2);
    uart_flush(UART_NUM_2);
    uart_set_baudrate(UART_NUM_2, 115200);

    ESP_LOGI(TAG, "UART2 monitor ready (GPIO35=TX, GPIO36=RX)");
    ESP_LOGI(TAG, "Monitoring ESP-AT output at 115200 baud...");

    char line[256];
    size_t n = 0;

    while (1) {
        int rx = uart_read_bytes(UART_NUM_2, buf, 1, 100 / portTICK_PERIOD_MS);
        if (rx == 1) {
            char ch = buf[0];
            if (ch == '\n' || ch == '\r') {
                if (n > 0) {
                    line[n] = '\0';
                    printf("[C6-AT] %s\n", line);

                    // Detect ready message from ESP-AT
                    if (strstr(line, "ready")) {
                        ESP_LOGI(TAG, "ESP-AT firmware is ready!");
                        ESP_LOGI(TAG, "You can now send AT commands via UART");
                    }
                    n = 0;
                }
            } else if (n < sizeof(line) - 1) {
                line[n++] = ch;
            } else {
                n = 0;  // overflow
            }
        }
    }
}

void app_main(void)
{
    esp_at_binaries_t bins;
    esp_loader_error_t err;

    // UART configuration for P4-Eye board
    // P4 UART2: TX=GPIO35, RX=GPIO36 connected to C6 UART
    // C6 control: EN=GPIO9, BOOT=GPIO33
    const loader_esp32_config_t config = {
        .baud_rate = 115200,
        .uart_port = UART_NUM_2,
        .uart_rx_pin = GPIO_NUM_36,    // P4 RX <- C6 TX
        .uart_tx_pin = GPIO_NUM_35,    // P4 TX -> C6 RX
        .reset_trigger_pin = GPIO_NUM_9,   // C6_EN
        .gpio0_trigger_pin = GPIO_NUM_33,  // C6_BOOT (IO0)
    };

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "  ESP-AT Firmware Flasher for ESP32-C6");
    ESP_LOGI(TAG, "  (UART Interface - More Reliable)");
    ESP_LOGI(TAG, "===========================================");

    if (loader_port_esp32_init(&config) != ESP_LOADER_SUCCESS) {
        ESP_LOGE(TAG, "UART init failed");
        abort();
    }

    if (connect_to_target(HIGHER_BAUDRATE) == ESP_LOADER_SUCCESS) {
        get_esp_at_binaries(&bins);

        ESP_LOGI(TAG, "Flashing 6 binaries for ESP-AT at %d baud...", HIGHER_BAUDRATE);

        // 1. Bootloader at 0x0
        ESP_LOGI(TAG, "[1/6] Flashing bootloader at 0x%x (%lu bytes)...",
                 (unsigned)bins.bootloader.addr, (unsigned long)bins.bootloader.size);
        err = flash_binary(bins.bootloader.data, bins.bootloader.size, bins.bootloader.addr);
        if (err != ESP_LOADER_SUCCESS) {
            ESP_LOGE(TAG, "Failed to flash bootloader!");
            goto done;
        }

        // 2. Partition table at 0x8000
        ESP_LOGI(TAG, "[2/6] Flashing partition table at 0x%x (%lu bytes)...",
                 (unsigned)bins.partition.addr, (unsigned long)bins.partition.size);
        err = flash_binary(bins.partition.data, bins.partition.size, bins.partition.addr);
        if (err != ESP_LOADER_SUCCESS) {
            ESP_LOGE(TAG, "Failed to flash partition table!");
            goto done;
        }

        // 3. OTA data at 0xd000
        ESP_LOGI(TAG, "[3/6] Flashing OTA data at 0x%x (%lu bytes)...",
                 (unsigned)bins.ota_data.addr, (unsigned long)bins.ota_data.size);
        err = flash_binary(bins.ota_data.data, bins.ota_data.size, bins.ota_data.addr);
        if (err != ESP_LOADER_SUCCESS) {
            ESP_LOGE(TAG, "Failed to flash OTA data!");
            goto done;
        }

        // 4. AT customize at 0x1e000
        ESP_LOGI(TAG, "[4/6] Flashing AT customize at 0x%x (%lu bytes)...",
                 (unsigned)bins.at_customize.addr, (unsigned long)bins.at_customize.size);
        err = flash_binary(bins.at_customize.data, bins.at_customize.size, bins.at_customize.addr);
        if (err != ESP_LOADER_SUCCESS) {
            ESP_LOGE(TAG, "Failed to flash AT customize!");
            goto done;
        }

        // 5. MFG NVS at 0x1f000
        ESP_LOGI(TAG, "[5/6] Flashing MFG NVS at 0x%x (%lu bytes)...",
                 (unsigned)bins.mfg_nvs.addr, (unsigned long)bins.mfg_nvs.size);
        err = flash_binary(bins.mfg_nvs.data, bins.mfg_nvs.size, bins.mfg_nvs.addr);
        if (err != ESP_LOADER_SUCCESS) {
            ESP_LOGE(TAG, "Failed to flash MFG NVS!");
            goto done;
        }

        // 6. ESP-AT app at 0x60000 (the large one - ~1.8MB)
        ESP_LOGI(TAG, "[6/6] Flashing ESP-AT app at 0x%x (%lu bytes)...",
                 (unsigned)bins.app.addr, (unsigned long)bins.app.size);
        err = flash_binary(bins.app.data, bins.app.size, bins.app.addr);
        if (err != ESP_LOADER_SUCCESS) {
            ESP_LOGE(TAG, "Failed to flash ESP-AT app!");
            goto done;
        }

        ESP_LOGI(TAG, "All binaries flashed successfully!");

done:
        ESP_LOGI(TAG, "Resetting target...");
        esp_loader_reset_target();

        vTaskDelay(500 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "***** ESP-AT Output (C6) *****");
        xTaskCreate(slave_monitor, "slave_monitor", 8192, NULL, configMAX_PRIORITIES - 1, NULL);
    }
    vTaskDelete(NULL);
}
