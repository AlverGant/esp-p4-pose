#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SDIO slave communication from ESP32-P4
 *
 * This replaces the UART communication with SDIO slave mode.
 * ESP32-C6 acts as SDIO slave, ESP32-P4 acts as SDIO host.
 */
esp_err_t coproc_sdio_slave_init(void);

/**
 * @brief Send a response line to ESP32-P4 via SDIO
 *
 * @param line Text line to send back to P4
 * @return ESP_OK on success, error code on failure
 */
esp_err_t coproc_sdio_slave_send_line(const char *line);

/**
 * @brief Start SDIO slave reception task
 *
 * This starts a task to receive messages from ESP32-P4
 * @param msg_handler Callback function to handle received messages
 * @return ESP_OK on success, error code on failure
 */
esp_err_t coproc_sdio_slave_start_rx(void (*msg_handler)(const char *message));

#ifdef __cplusplus
}
#endif