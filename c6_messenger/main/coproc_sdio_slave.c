#include "coproc_sdio_slave.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/sdio_slave.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG_SDIO = "sdio_slave";

// SDIO message structure (same as host)
typedef struct {
    uint32_t magic;     // 0xFA112024
    uint32_t length;    // Message length
    char data[256];     // Message data
    uint32_t checksum;  // Simple checksum
} __attribute__((packed)) sdio_message_t;

#define SDIO_MSG_MAGIC 0xFA112024
#define SDIO_SLAVE_QUEUE_SIZE 8

static bool s_sdio_slave_inited = false;
static TaskHandle_t s_rx_task = NULL;
static void (*s_msg_handler)(const char *) = NULL;
static QueueHandle_t s_tx_queue = NULL;

static uint32_t calculate_checksum(const sdio_message_t *msg)
{
    uint32_t sum = msg->magic + msg->length;
    for (size_t i = 0; i < msg->length && i < sizeof(msg->data); i++) {
        sum += (uint8_t)msg->data[i];
    }
    return sum;
}

static void sdio_slave_tx_task(void *arg)
{
    (void)arg;
    sdio_message_t tx_msg;

    ESP_LOGI(TAG_SDIO, "SDIO slave TX task started");

    for (;;) {
        if (xQueueReceive(s_tx_queue, &tx_msg, portMAX_DELAY) == pdTRUE) {
            // Send the message - cast to uint8_t* for sdio_slave_transmit
            esp_err_t ret = sdio_slave_transmit((uint8_t*)&tx_msg, sizeof(tx_msg));
            if (ret == ESP_OK) {
                ESP_LOGI(TAG_SDIO, "SDIO response sent: %s", tx_msg.data);
            } else {
                ESP_LOGW(TAG_SDIO, "Failed to send SDIO response: %s", esp_err_to_name(ret));
            }
        }
    }
}

static void sdio_slave_rx_task(void *arg)
{
    (void)arg;
    sdio_message_t rx_msg;
    sdio_slave_buf_handle_t handle = NULL;
    uint8_t *recv_buf = NULL;
    size_t length = 0;

    ESP_LOGI(TAG_SDIO, "SDIO slave RX task started");

    for (;;) {
        // Receive from SDIO - using correct API signature
        esp_err_t ret = sdio_slave_recv(&handle, &recv_buf, &length, portMAX_DELAY);

        if (ret == ESP_OK && recv_buf != NULL && length >= sizeof(rx_msg)) {
            // Copy received data to message structure
            memcpy(&rx_msg, recv_buf, sizeof(rx_msg));

            // Return the buffer via handle
            ret = sdio_slave_recv_load_buf(handle);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG_SDIO, "Failed to reload buffer: %s", esp_err_to_name(ret));
            }
            handle = NULL;
            recv_buf = NULL;

            // Validate message
            if (rx_msg.magic == SDIO_MSG_MAGIC &&
                rx_msg.length < sizeof(rx_msg.data) &&
                calculate_checksum(&rx_msg) == rx_msg.checksum) {

                rx_msg.data[rx_msg.length] = '\0';  // Ensure null termination
                ESP_LOGI(TAG_SDIO, "SDIO message received: %s", rx_msg.data);

                // Call message handler
                if (s_msg_handler) {
                    s_msg_handler(rx_msg.data);
                }

                // Send acknowledgment
                coproc_sdio_slave_send_line("C6: Message received");
            } else {
                ESP_LOGW(TAG_SDIO, "Invalid SDIO message (magic=0x%08lX, len=%lu, size=%zu)",
                        (unsigned long)rx_msg.magic, (unsigned long)rx_msg.length, length);
            }
        } else if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG_SDIO, "SDIO receive failed: %s", esp_err_to_name(ret));
            if (handle) {
                sdio_slave_recv_load_buf(handle);
                handle = NULL;
            }
            vTaskDelay(pdMS_TO_TICKS(100));  // Brief delay on error
        }
    }
}

esp_err_t coproc_sdio_slave_init(void)
{
    if (s_sdio_slave_inited) {
        ESP_LOGI(TAG_SDIO, "SDIO slave already initialized");
        return ESP_OK;
    }

    // ESP32-C6 SDIO slave pins are hardware fixed:
    // CLK=GPIO19, CMD=GPIO18, D0=GPIO20, D1=GPIO21, D2=GPIO22, D3=GPIO23
    ESP_LOGI(TAG_SDIO, "Initializing SDIO slave (fixed pins: CLK=19, CMD=18, D0=20, D1=21, D2=22, D3=23)...");

    // Configure SDIO slave (use PACKET mode like example, not STREAM)
    sdio_slave_config_t slave_config = {
        .sending_mode = SDIO_SLAVE_SEND_PACKET,  // Changed from STREAM to PACKET
        .send_queue_size = SDIO_SLAVE_QUEUE_SIZE,
        .recv_buffer_size = sizeof(sdio_message_t),  // Single message size
        .event_cb = NULL,  // No event callback for now
        .flags = 0,
    };

    esp_err_t ret = sdio_slave_initialize(&slave_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SDIO, "Failed to initialize SDIO slave: %s", esp_err_to_name(ret));
        return ret;
    }

    // Allocate and register receive buffers
    const size_t buffer_size = sizeof(sdio_message_t);
    for (int i = 0; i < SDIO_SLAVE_QUEUE_SIZE; i++) {
        uint8_t *recv_buf = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
        if (!recv_buf) {
            ESP_LOGE(TAG_SDIO, "Failed to allocate DMA buffer %d", i);
            return ESP_ERR_NO_MEM;
        }
        sdio_slave_buf_handle_t handle = sdio_slave_recv_register_buf(recv_buf);
        if (handle == NULL) {
            ESP_LOGE(TAG_SDIO, "Failed to register buffer %d", i);
            free(recv_buf);
            return ESP_ERR_NO_MEM;
        }
        // IMPORTANT: Load buffer into receive queue like the example does!
        ret = sdio_slave_recv_load_buf(handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG_SDIO, "Failed to load buffer %d: %s", i, esp_err_to_name(ret));
            return ret;
        }
    }

    // Initialize SDIO registers (required by ESSL protocol)
    // Register 0 is used for job/status communication with host
    sdio_slave_write_reg(0, 0);  // Initialize to idle state

    // Initialize other registers to known values (ESSL may check these)
    for (int i = 1; i < 60; i++) {
        int addr = (i >= 28) ? (i + 4) : i;  // Skip interrupt registers
        sdio_slave_write_reg(addr, 0);
    }

    ESP_LOGI(TAG_SDIO, "SDIO registers initialized");

    // Enable host interrupts (CRITICAL - example does this!)
    sdio_slave_set_host_intena(SDIO_SLAVE_HOSTINT_SEND_NEW_PACKET |
                               SDIO_SLAVE_HOSTINT_BIT0 |
                               SDIO_SLAVE_HOSTINT_BIT1 |
                               SDIO_SLAVE_HOSTINT_BIT2 |
                               SDIO_SLAVE_HOSTINT_BIT3 |
                               SDIO_SLAVE_HOSTINT_BIT4 |
                               SDIO_SLAVE_HOSTINT_BIT5 |
                               SDIO_SLAVE_HOSTINT_BIT6 |
                               SDIO_SLAVE_HOSTINT_BIT7);

    // Start SDIO slave AFTER everything is configured
    ret = sdio_slave_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SDIO, "Failed to start SDIO slave: %s", esp_err_to_name(ret));
        sdio_slave_stop();
        return ret;
    }

    // Create TX queue
    s_tx_queue = xQueueCreate(SDIO_SLAVE_QUEUE_SIZE, sizeof(sdio_message_t));
    if (!s_tx_queue) {
        ESP_LOGE(TAG_SDIO, "Failed to create SDIO TX queue");
        sdio_slave_stop();
        return ESP_ERR_NO_MEM;
    }

    // Create TX task
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        sdio_slave_tx_task, "sdio_tx", 3072, NULL, 5, NULL, 0);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG_SDIO, "Failed to create SDIO TX task");
        vQueueDelete(s_tx_queue);
        sdio_slave_stop();
        return ESP_ERR_NO_MEM;
    }

    s_sdio_slave_inited = true;
    ESP_LOGI(TAG_SDIO, "SDIO slave initialized successfully");
    return ESP_OK;
}

esp_err_t coproc_sdio_slave_send_line(const char *line)
{
    if (!line || !s_sdio_slave_inited || !s_tx_queue) {
        return ESP_ERR_INVALID_ARG;
    }

    // Prepare message
    sdio_message_t msg = {0};
    msg.magic = SDIO_MSG_MAGIC;
    msg.length = strnlen(line, sizeof(msg.data) - 1);
    strncpy(msg.data, line, sizeof(msg.data) - 1);
    msg.data[msg.length] = '\0';
    msg.checksum = calculate_checksum(&msg);

    // Queue message for transmission
    if (xQueueSend(s_tx_queue, &msg, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG_SDIO, "Failed to queue SDIO message for transmission");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t coproc_sdio_slave_start_rx(void (*msg_handler)(const char *message))
{
    if (!s_sdio_slave_inited) {
        ESP_LOGW(TAG_SDIO, "SDIO slave not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_rx_task) {
        ESP_LOGI(TAG_SDIO, "SDIO slave RX task already running");
        return ESP_OK;
    }

    s_msg_handler = msg_handler;

    BaseType_t task_ret = xTaskCreatePinnedToCore(
        sdio_slave_rx_task, "sdio_rx", 4096, NULL, 6, &s_rx_task, 0);

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG_SDIO, "Failed to create SDIO RX task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG_SDIO, "SDIO slave RX task started");
    return ESP_OK;
}