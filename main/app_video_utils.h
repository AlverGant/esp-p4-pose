#pragma once

#include "esp_err.h"
#include "driver/ppa.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t app_video_utils_init(void);
esp_err_t app_video_utils_deinit(void);

esp_err_t app_image_process_scale_crop(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    uint32_t crop_width, uint32_t crop_height,
    uint8_t *out_buf, uint32_t out_width, uint32_t out_height, size_t out_buf_size,
    ppa_srm_rotation_angle_t rotation_angle);


void swap_rgb565_bytes(uint16_t *buffer, int pixel_count);

#ifdef __cplusplus
}
#endif
