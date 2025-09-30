#include <stdio.h>
#include "esp_log.h"
#include "driver/ppa.h"
#include "driver/jpeg_encode.h"
#include "bsp/display.h"

#include "app_video_utils.h"

static const char *TAG = "app_video_utils";

static ppa_client_handle_t ppa_srm_handle = NULL;

esp_err_t app_video_utils_init(void)
{
    ppa_client_config_t ppa_srm_config = {
        .oper_type = PPA_OPERATION_SRM,
    };
    esp_err_t ret = ppa_register_client(&ppa_srm_config, &ppa_srm_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register PPA client: 0x%x", ret);
    }
    return ret;
}

esp_err_t app_video_utils_deinit(void)
{
    if (ppa_srm_handle) {
        ppa_unregister_client(ppa_srm_handle);
        ppa_srm_handle = NULL;
    }
    return ESP_OK;
}

esp_err_t app_image_process_scale_crop(
    uint8_t *in_buf, uint32_t in_width, uint32_t in_height,
    uint32_t crop_width, uint32_t crop_height,
    uint8_t *out_buf, uint32_t out_width, uint32_t out_height, size_t out_buf_size,
    ppa_srm_rotation_angle_t rotation_angle)
{
    ppa_srm_oper_config_t srm_config = {
        .in.buffer = in_buf,
        .in.pic_w = in_width,
        .in.pic_h = in_height,
        .in.block_w = crop_width,
        .in.block_h = crop_height,
        .in.block_offset_x = (in_width - crop_width) / 2,
        .in.block_offset_y = (in_height - crop_height) / 2,
        .in.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        .out.buffer = out_buf,
        .out.buffer_size = out_buf_size,
        .out.pic_w = out_width,
        .out.pic_h = out_height,
        .out.block_offset_x = 0,
        .out.block_offset_y = 0,
        .out.srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        .rotation_angle = rotation_angle,
        .scale_x = (float)out_width / crop_width,
        .scale_y = (float)out_height / crop_height,
        .rgb_swap = 0,
        .byte_swap = 0,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    return ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
}


void swap_rgb565_bytes(uint16_t *buffer, int pixel_count)
{
    for (int i = 0; i < pixel_count; i++) {
        uint16_t v = buffer[i];
        buffer[i] = (v >> 8) | (v << 8);
    }
}
