#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send photo to Telegram via C6 coprocessor
 *
 * Encodes RGB565 frame to JPEG and sends to C6 via UART for Telegram upload
 *
 * @param rgb565_frame Pointer to RGB565 frame buffer
 * @param width Frame width in pixels
 * @param height Frame height in pixels
 * @param caption Optional caption text (can be NULL)
 * @return ESP_OK on success
 */
esp_err_t telegram_send_photo_from_frame(const uint16_t *rgb565_frame, int width, int height, const char *caption);

#ifdef __cplusplus
}
#endif
