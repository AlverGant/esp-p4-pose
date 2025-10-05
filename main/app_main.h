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
 * LOCKS the buffer - caller MUST call app_main_release_frame() when done!
 *
 * @param width Output: frame width (typically 240)
 * @param height Output: frame height (typically 240)
 * @return Pointer to RGB565 frame buffer, or NULL if not ready
 */
const uint16_t* app_main_get_latest_frame(int *width, int *height);

/**
 * @brief Release the frame buffer locked by app_main_get_latest_frame()
 *
 * MUST be called after photo capture is complete to allow camera to continue.
 */
void app_main_release_frame(void);

#ifdef __cplusplus
}
#endif
