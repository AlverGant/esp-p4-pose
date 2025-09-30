#include "coproc_sdio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "esp_serial_slave_link/essl_sdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG_SDIO = "coproc_sdio";

// ESP32-P4-Eye SDIO pin configuration
#define SDIO_CLK_GPIO    28
#define SDIO_CMD_GPIO    27
#define SDIO_D0_GPIO     29
#define SDIO_D1_GPIO     30
#define SDIO_D2_GPIO     31
#define SDIO_D3_GPIO     32

// C6 control pins
#define C6_RESET_GPIO    9   // C6_EN
#define C6_BOOT_GPIO     33  // C6_BOOT (IO0)

static sdmmc_card_t *s_card = NULL;
static essl_handle_t s_essl_handle = NULL;
static bool s_sdio_inited = false;
static TaskHandle_t s_rx_task = NULL;

// SDIO message structure (align payload to 512 bytes for SDMMC transfers)
#define SDIO_MSG_TOTAL_SIZE   512
#define SDIO_MSG_MAGIC        0xFA112024
#define SDIO_MSG_DATA_MAX_LEN (SDIO_MSG_TOTAL_SIZE - sizeof(uint32_t) * 3)

typedef struct {
    uint32_t magic;     // 0xFA112024
    uint32_t length;    // Message length (bytes used in data[])
    char data[SDIO_MSG_DATA_MAX_LEN];
    uint32_t checksum;  // Simple checksum
} __attribute__((packed)) sdio_message_t;

_Static_assert(sizeof(sdio_message_t) == SDIO_MSG_TOTAL_SIZE, "SDIO message size must stay 512 bytes");

static uint32_t calculate_checksum(const sdio_message_t *msg)
{
    uint32_t sum = msg->magic + msg->length;
    for (size_t i = 0; i < msg->length && i < sizeof(msg->data); i++) {
        sum += (uint8_t)msg->data[i];
    }
    return sum;
}

static esp_err_t sdio_host_init(void)
{
    // Reset C6 to ensure it boots with latest firmware
    ESP_LOGI(TAG_SDIO, "Resetting C6 coprocessor...");
    gpio_reset_pin(C6_BOOT_GPIO);
    gpio_set_direction(C6_BOOT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(C6_BOOT_GPIO, 1);  // Ensure normal boot mode (HIGH = boot from flash)

    gpio_reset_pin(C6_RESET_GPIO);
    gpio_set_direction(C6_RESET_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(C6_RESET_GPIO, 0);  // Assert reset (LOW)
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(C6_RESET_GPIO, 1);  // Release reset (HIGH)

    ESP_LOGI(TAG_SDIO, "Waiting for C6 to boot and initialize SDIO slave (8 seconds)...");
    vTaskDelay(pdMS_TO_TICKS(8000));  // Wait for C6 boot and SDIO slave init
    ESP_LOGI(TAG_SDIO, "C6 should be ready, proceeding with SDIO host init");

    // Initialize SDMMC host
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_1;  // Use slot 1 for ESP32-P4-Eye
    host.max_freq_khz = 20000;      // 20MHz initially, can increase later
    host.flags = SDMMC_HOST_FLAG_4BIT;  // 4-bit mode

    // Configure SDIO pins for ESP32-P4-Eye
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = SDIO_CLK_GPIO;
    slot_config.cmd = SDIO_CMD_GPIO;
    slot_config.d0 = SDIO_D0_GPIO;
    slot_config.d1 = SDIO_D1_GPIO;
    slot_config.d2 = SDIO_D2_GPIO;
    slot_config.d3 = SDIO_D3_GPIO;
    slot_config.width = 4;  // 4-bit SDIO

    ESP_LOGI(TAG_SDIO, "Initializing SDIO host: CLK=%d, CMD=%d, D0=%d, D1=%d, D2=%d, D3=%d",
             SDIO_CLK_GPIO, SDIO_CMD_GPIO, SDIO_D0_GPIO, SDIO_D1_GPIO, SDIO_D2_GPIO, SDIO_D3_GPIO);

    esp_err_t ret = sdmmc_host_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SDIO, "Failed to initialize SDMMC host: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = sdmmc_host_init_slot(host.slot, &slot_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SDIO, "Failed to initialize SDMMC slot: %s", esp_err_to_name(ret));
        sdmmc_host_deinit();
        return ret;
    }

    // Allocate card structure
    s_card = malloc(sizeof(sdmmc_card_t));
    if (!s_card) {
        ESP_LOGE(TAG_SDIO, "Failed to allocate SDIO card structure");
        sdmmc_host_deinit();
        return ESP_ERR_NO_MEM;
    }

    // Initialize card structure
    memset(s_card, 0, sizeof(sdmmc_card_t));
    s_card->host = host;
    s_card->is_sdio = 1;  // Mark as SDIO card
    s_card->is_mmc = 0;

    // Probe and initialize SDIO slave device (C6) with retries
    // C6 takes ~8s to boot and initialize SDIO slave, so retry multiple times
    const int max_retries = 20;  // 20 retries = 10 seconds max
    for (int i = 0; i < max_retries; i++) {
        ret = sdmmc_card_init(&host, s_card);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG_SDIO, "SDIO slave device detected and initialized (attempt %d/%d)", i+1, max_retries);

            // Initialize ESSL (ESP Serial Slave Link) for high-level packet communication
            essl_sdio_config_t essl_config = {
                .card = s_card,
                // Host and slave must agree on the packet buffer size. We use the
                // exact payload size handled by the C6 firmware to avoid the ESSL
                // driver mis-counting buffers (it treats this value as the
                // negotiated buffer length).
                .recv_buffer_size = sizeof(sdio_message_t),
            };

            ret = essl_sdio_init_dev(&s_essl_handle, &essl_config);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG_SDIO, "Failed to initialize ESSL: %s", esp_err_to_name(ret));
                return ret;
            }

            ESP_LOGI(TAG_SDIO, "ESSL device created successfully");

            // Initialize the ESSL device (required before sending/receiving)
            ret = essl_init(s_essl_handle, 5000);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG_SDIO, "Failed to init ESSL device: %s", esp_err_to_name(ret));
                return ret;
            }

            ESP_LOGI(TAG_SDIO, "ESSL initialized successfully");

            // Wait for the slave to be ready
            ret = essl_wait_for_ready(s_essl_handle, 5000);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG_SDIO, "Slave not ready after init: %s (continuing anyway)", esp_err_to_name(ret));
            } else {
                ESP_LOGI(TAG_SDIO, "ESSL slave is ready");
            }

            ESP_LOGI(TAG_SDIO, "SDIO host initialized successfully");
            return ESP_OK;
        }

        if (i == 0) {
            ESP_LOGI(TAG_SDIO, "Waiting for C6 SDIO slave to be ready...");
        }

        if (i < max_retries - 1) {
            vTaskDelay(pdMS_TO_TICKS(500));  // Wait 500ms between retries
        }
    }

    ESP_LOGE(TAG_SDIO, "SDIO card init failed after %d retries: %s", max_retries, esp_err_to_name(ret));
    if (s_card) {
        free(s_card);
        s_card = NULL;
    }
    sdmmc_host_deinit();
    return ret;
}

static void coproc_sdio_rx_task(void *arg)
{
    (void)arg;
    if (!s_essl_handle) {
        ESP_LOGE(TAG_SDIO, "ESSL handle not initialized for RX task");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG_SDIO, "SDIO RX task started");

    const size_t packet_size = sizeof(sdio_message_t);
    uint8_t *buffer = heap_caps_malloc(packet_size, MALLOC_CAP_DMA);
    if (!buffer) {
        ESP_LOGE(TAG_SDIO, "Failed to allocate DMA buffer for SDIO RX");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        // Try to receive packet from SDIO slave using ESSL API
        size_t size_read = packet_size;
        const int wait_ms = 1000;  // 1 second timeout
        esp_err_t ret = essl_get_packet(s_essl_handle, buffer, packet_size, &size_read, wait_ms);

        if (ret == ESP_OK) {
            // Validate message
            sdio_message_t *rx_msg = (sdio_message_t*)buffer;
            if (size_read >= sizeof(sdio_message_t) &&
                rx_msg->magic == SDIO_MSG_MAGIC &&
                rx_msg->length < sizeof(rx_msg->data) &&
                calculate_checksum(rx_msg) == rx_msg->checksum) {

                rx_msg->data[rx_msg->length] = '\0';
                ESP_LOGI(TAG_SDIO, "SDIO RX: %s", rx_msg->data);
            } else {
                ESP_LOGD(TAG_SDIO, "Invalid SDIO message received (size=%zu)", size_read);
            }
        } else if (ret == ESP_ERR_NOT_FOUND) {
            // No data available, this is normal - just wait
            vTaskDelay(pdMS_TO_TICKS(500));
        } else if (ret == ESP_ERR_TIMEOUT) {
            // Timeout is normal when no data - just continue
            continue;
        } else if (ret == ESP_ERR_INVALID_ARG) {
            // Invalid arg error - log once and increase delay to avoid spam
            static bool logged_once = false;
            if (!logged_once) {
                ESP_LOGW(TAG_SDIO, "SDIO RX: ESP_ERR_INVALID_ARG (will suppress further logs)");
                logged_once = true;
            }
            vTaskDelay(pdMS_TO_TICKS(5000));  // Wait 5 seconds before retry
        } else {
            ESP_LOGW(TAG_SDIO, "SDIO RX error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

esp_err_t coproc_sdio_init(void)
{
#if !CONFIG_COPROC_UART_ENABLE
    ESP_LOGW(TAG_SDIO, "COPROC communication disabled in config");
    return ESP_ERR_INVALID_STATE;
#endif

    if (s_sdio_inited) {
        ESP_LOGI(TAG_SDIO, "SDIO already initialized");
        return ESP_OK;
    }

    esp_err_t ret = sdio_host_init();
    if (ret != ESP_OK) {
        return ret;
    }

    s_sdio_inited = true;
    ESP_LOGI(TAG_SDIO, "SDIO coprocessor communication ready");
    return ESP_OK;
}

esp_err_t coproc_sdio_send_line(const char *line)
{
#if !CONFIG_COPROC_UART_ENABLE
    return ESP_ERR_INVALID_STATE;
#endif

    if (!line) {
        ESP_LOGE(TAG_SDIO, "Line is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_essl_handle) {
        ESP_LOGE(TAG_SDIO, "ESSL handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_sdio_inited) {
        ESP_LOGW(TAG_SDIO, "SDIO not initialized, skipping send");
        return ESP_ERR_INVALID_STATE;
    }

    // Prepare SDIO message
    sdio_message_t msg = {0};
    msg.magic = SDIO_MSG_MAGIC;
    msg.length = strnlen(line, sizeof(msg.data) - 1);
    strncpy(msg.data, line, sizeof(msg.data) - 1);
    msg.data[msg.length] = '\0';
    msg.checksum = calculate_checksum(&msg);

    ESP_LOGI(TAG_SDIO, "Sending SDIO packet: %zu bytes", sizeof(sdio_message_t));

    // Check if handle is valid
    if (!s_essl_handle) {
        ESP_LOGE(TAG_SDIO, "ESSL handle is NULL!");
        return ESP_ERR_INVALID_ARG;
    }

    // Check buffer availability first
    uint32_t tx_num = 0;
    esp_err_t ret = essl_get_tx_buffer_num(s_essl_handle, &tx_num, 100);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_SDIO, "C6 has %lu TX buffers available", (unsigned long)tx_num);
    } else {
        ESP_LOGW(TAG_SDIO, "Failed to get TX buffer num: %s", esp_err_to_name(ret));
    }

    // Send via ESSL packet API (handles SDIO packet mode protocol)
    const int wait_ms = 1000;  // 1 second timeout
    ret = essl_send_packet(s_essl_handle, (uint8_t*)&msg, sizeof(sdio_message_t), wait_ms);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG_SDIO, "SDIO TX: '%s'", line);
        return ESP_OK;
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG_SDIO, "SDIO send timeout (slave not ready?)");
        return ret;
    } else {
        ESP_LOGW(TAG_SDIO, "SDIO send failed: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }
}

esp_err_t coproc_sdio_start_rx_log(void)
{
#if !CONFIG_COPROC_UART_ENABLE
    return ESP_ERR_INVALID_STATE;
#endif

    if (s_rx_task) {
        ESP_LOGI(TAG_SDIO, "SDIO RX task already running");
        return ESP_OK;
    }

    if (!s_sdio_inited) {
        ESP_LOGW(TAG_SDIO, "SDIO not initialized, cannot start RX task");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        coproc_sdio_rx_task, "coproc_sdio_rx", 4096, NULL, 2, &s_rx_task, 0);

    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}