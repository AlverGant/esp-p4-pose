#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get the latest camera frame for photo capture
 *
 * Returns a pointer to the latest RGB565 frame buffer displayed on LCD.
 * This is safe to call from other tasks for photo capture.
 *
 * @param width Output: frame width (typically 240)
 * @param height Output: frame height (typically 240)
 * @return Pointer to RGB565 frame buffer, or NULL if not ready
 */
const uint16_t* app_main_get_latest_frame(int *width, int *height);

#ifdef __cplusplus
}
#endif
