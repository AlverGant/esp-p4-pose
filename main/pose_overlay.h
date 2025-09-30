#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t pose_overlay_init(void);
// Submit a frame for background inference (non-blocking)
esp_err_t pose_overlay_submit(const uint16_t *rgb565_be_buf, int width, int height);
// Draw last pose result overlay onto the buffer (fast)
esp_err_t pose_overlay_draw(uint16_t *rgb565_be_buf, int width, int height);
// Check if fall is detected
bool pose_overlay_is_fall_detected(void);

// Stats about last inference
void pose_overlay_get_stats(int *persons, int *age_ms, int *seq);

// Inform pose overlay of the rotation applied to pose input (degrees: 0/90/180/270)
void pose_overlay_set_input_rotation(int rot_deg);

// Get last ROI (normalized to [0,1] over pose input W/H)
// Returns true if valid; age_ms gives time since computed

#ifdef __cplusplus
}
#endif
