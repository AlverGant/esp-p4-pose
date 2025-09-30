#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SDIO host communication to ESP32-C6
 *
 * This replaces the UART communication with SDIO host mode.
 * ESP32-P4 acts as SDIO host, ESP32-C6 acts as SDIO slave.
 */
esp_err_t coproc_sdio_init(void);

/**
 * @brief Send a text line to ESP32-C6 via SDIO
 *
 * @param line Text line to send (will be null-terminated automatically)
 * @return ESP_OK on success, error code on failure
 */
esp_err_t coproc_sdio_send_line(const char *line);

/**
 * @brief Start SDIO reception logging task
 *
 * This starts a task to receive and log responses from ESP32-C6
 * @return ESP_OK on success, error code on failure
 */
esp_err_t coproc_sdio_start_rx_log(void);

#ifdef __cplusplus
}
#endif