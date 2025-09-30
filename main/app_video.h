/* Minimal subset from factory_demo to open V4L2 camera and stream frames */
#pragma once

#include "esp_err.h"
#include "linux/videodev2.h"
#include "esp_video_device.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    APP_VIDEO_FMT_RAW8 = V4L2_PIX_FMT_SBGGR8,
    APP_VIDEO_FMT_RAW10 = V4L2_PIX_FMT_SBGGR10,
    APP_VIDEO_FMT_GREY = V4L2_PIX_FMT_GREY,
    APP_VIDEO_FMT_RGB565 = V4L2_PIX_FMT_RGB565,
    APP_VIDEO_FMT_RGB888 = V4L2_PIX_FMT_RGB24,
    APP_VIDEO_FMT_YUV422 = V4L2_PIX_FMT_YUV422P,
    APP_VIDEO_FMT_YUV420 = V4L2_PIX_FMT_YUV420,
} video_fmt_t;

#define EXAMPLE_CAM_DEV_PATH   (ESP_VIDEO_MIPI_CSI_DEVICE_NAME)
#define EXAMPLE_CAM_BUF_NUM    (2)

#define APP_VIDEO_FMT          (APP_VIDEO_FMT_RGB565)

typedef void (*app_video_frame_operation_cb_t)(uint8_t *camera_buf, uint8_t camera_buf_index, uint32_t camera_buf_hes, uint32_t camera_buf_ves, size_t camera_buf_len);

esp_err_t app_video_main(i2c_master_bus_handle_t i2c_bus_handle);
int app_video_open(char *dev, video_fmt_t init_fmt);
esp_err_t app_video_set_bufs(int video_fd, uint32_t fb_num, const void **fb);
uint32_t app_video_get_buf_size(void);
esp_err_t app_video_stream_task_start(int video_fd, int core_id);
esp_err_t app_video_stream_task_stop(int video_fd);
esp_err_t app_video_wait_video_stop(void);
esp_err_t app_video_register_frame_operation_cb(app_video_frame_operation_cb_t operation_cb);
esp_err_t app_video_close(int video_fd);

// Camera controls (V4L2). All functions are best-effort; they log and continue on failure.
esp_err_t app_video_set_control(int video_fd, uint32_t id, int32_t value);
esp_err_t app_video_get_control(int video_fd, uint32_t id, int32_t *out_value);
// Set a control using a normalized 0..1 value by mapping to (min..max) from QUERYCTRL
esp_err_t app_video_set_control_normalized(int video_fd, uint32_t id, float norm_value);
// Check if a control exists and is enabled
bool app_video_has_control(int video_fd, uint32_t id);
// Convenience: boost overall contrast/color edges for pose
void app_video_apply_pose_tuning(int video_fd);

#ifdef __cplusplus
}
#endif
