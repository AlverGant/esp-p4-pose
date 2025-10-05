/*
 * Minimal copy of the video device glue used by esp-video
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "esp_err.h"
#include "esp_log.h"
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "app_video.h"

static const char *TAG = "app_video";

#define MAX_BUFFER_COUNT                (6)
#define MIN_BUFFER_COUNT                (2)
#define VIDEO_TASK_STACK_SIZE           (4 * 1024)
#define VIDEO_TASK_PRIORITY             (6)

typedef struct {
    uint8_t *camera_buffer[MAX_BUFFER_COUNT];
    size_t camera_buf_size;
    uint32_t camera_buf_hes;
    uint32_t camera_buf_ves;
    struct v4l2_buffer v4l2_buf;
    uint8_t camera_mem_mode;
    app_video_frame_operation_cb_t user_camera_video_frame_operation_cb;
    TaskHandle_t video_stream_task_handle;
    bool video_task_delete;
    SemaphoreHandle_t video_stop_sem;
} app_video_t;

static app_video_t app_camera_video;

esp_err_t app_video_main(i2c_master_bus_handle_t i2c_bus_handle)
{
    const esp_video_init_csi_config_t base_csi_config = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port      = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_PORT,
                .scl_pin   = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SCL_PIN,
                .sda_pin   = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SDA_PIN,
            },
            .freq      = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_FREQ,
        },
        .reset_pin = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_RESET_PIN,
        .pwdn_pin  = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_PWDN_PIN,
    };

    esp_video_init_csi_config_t csi_config = base_csi_config;
    if (i2c_bus_handle != NULL) {
        csi_config.sccb_config.init_sccb = false;
        csi_config.sccb_config.i2c_handle = i2c_bus_handle;
    }

    esp_video_init_config_t cam_config = {
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR > 0
        .csi      = &csi_config,
#endif
    };

    return esp_video_init(&cam_config);
}

int app_video_open(char *dev, video_fmt_t init_fmt)
{
    struct v4l2_format default_format;
    struct v4l2_capability capability;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    int fd = open(dev, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "Open video failed");
        return -1;
    }

    if (ioctl(fd, VIDIOC_QUERYCAP, &capability)) {
        ESP_LOGE(TAG, "failed to get capability");
        goto exit_0;
    }

    ESP_LOGI(TAG, "driver: %s | card: %s", capability.driver, capability.card);

    memset(&default_format, 0, sizeof(struct v4l2_format));
    default_format.type = type;
    if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
        ESP_LOGE(TAG, "failed to get format");
        goto exit_0;
    }

    app_camera_video.camera_buf_hes = default_format.fmt.pix.width;
    app_camera_video.camera_buf_ves = default_format.fmt.pix.height;

    if (default_format.fmt.pix.pixelformat != init_fmt) {
        struct v4l2_format format = {
            .type = type,
            .fmt.pix.width = default_format.fmt.pix.width,
            .fmt.pix.height = default_format.fmt.pix.height,
            .fmt.pix.pixelformat = init_fmt,
        };

        if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
            ESP_LOGE(TAG, "failed to set format");
            goto exit_0;
        }
    }

    app_camera_video.video_stop_sem = xSemaphoreCreateBinary();

    return fd;
exit_0:
    close(fd);
    return -1;
}

esp_err_t app_video_set_bufs(int video_fd, uint32_t fb_num, const void **fb)
{
    if (fb_num > MAX_BUFFER_COUNT) {
        ESP_LOGE(TAG, "buffer num is too large");
        return ESP_FAIL;
    } else if (fb_num < MIN_BUFFER_COUNT) {
        ESP_LOGE(TAG, "At least two buffers are required");
        return ESP_FAIL;
    }

    struct v4l2_requestbuffers req;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    memset(&req, 0, sizeof(req));
    req.count = fb_num;
    req.type = type;

    app_camera_video.camera_mem_mode = req.memory = fb ? V4L2_MEMORY_USERPTR : V4L2_MEMORY_MMAP;

    if (ioctl(video_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "req bufs failed");
        goto errout_req_bufs;
    }
    for (int i = 0; i < fb_num; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = type;
        buf.memory = req.memory;
        buf.index = i;

        if (ioctl(video_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "query buf failed");
            goto errout_req_bufs;
        }

        if (req.memory == V4L2_MEMORY_MMAP) {
            app_camera_video.camera_buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, video_fd, buf.m.offset);
            if (app_camera_video.camera_buffer[i] == NULL) {
                ESP_LOGE(TAG, "mmap failed");
                goto errout_req_bufs;
            }
        } else {
            if (!fb[i]) {
                ESP_LOGE(TAG, "frame buffer is NULL");
                goto errout_req_bufs;
            }
            buf.m.userptr = (unsigned long)fb[i];
            app_camera_video.camera_buffer[i] = (uint8_t *)fb[i];
        }

        app_camera_video.camera_buf_size = buf.length;

        if (ioctl(video_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "queue frame buffer failed");
            goto errout_req_bufs;
        }
    }

    return ESP_OK;

errout_req_bufs:
    close(video_fd);
    return ESP_FAIL;
}

uint32_t app_video_get_buf_size(void)
{
    uint32_t bpp = (APP_VIDEO_FMT == APP_VIDEO_FMT_RGB565 ? 2 : 3);
    return app_camera_video.camera_buf_hes * app_camera_video.camera_buf_ves * bpp;
}

static inline esp_err_t video_receive_video_frame(int video_fd)
{
    memset(&app_camera_video.v4l2_buf, 0, sizeof(app_camera_video.v4l2_buf));
    app_camera_video.v4l2_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    app_camera_video.v4l2_buf.memory = app_camera_video.camera_mem_mode;
    int res = ioctl(video_fd, VIDIOC_DQBUF, &(app_camera_video.v4l2_buf));
    return (res == 0) ? ESP_OK : ESP_FAIL;
}

static inline void video_operation_video_frame(int video_fd)
{
    app_camera_video.v4l2_buf.m.userptr = (unsigned long)app_camera_video.camera_buffer[app_camera_video.v4l2_buf.index];
    app_camera_video.v4l2_buf.length = app_camera_video.camera_buf_size;

    uint8_t buf_index = app_camera_video.v4l2_buf.index;

    if (app_camera_video.user_camera_video_frame_operation_cb) {
        app_camera_video.user_camera_video_frame_operation_cb(
            app_camera_video.camera_buffer[buf_index],
            buf_index,
            app_camera_video.camera_buf_hes,
            app_camera_video.camera_buf_ves,
            app_camera_video.camera_buf_size
        );
    }
}

static inline esp_err_t video_free_video_frame(int video_fd)
{
    return (ioctl(video_fd, VIDIOC_QBUF, &(app_camera_video.v4l2_buf)) == 0) ? ESP_OK : ESP_FAIL;
}

static inline esp_err_t video_stream_start(int video_fd)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return (ioctl(video_fd, VIDIOC_STREAMON, &type) == 0) ? ESP_OK : ESP_FAIL;
}

static inline esp_err_t video_stream_stop(int video_fd)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return (ioctl(video_fd, VIDIOC_STREAMOFF, &type) == 0) ? ESP_OK : ESP_FAIL;
}

static void video_stream_task(void *arg)
{
    int video_fd = *((int *)arg);
    while (1) {
        if (video_receive_video_frame(video_fd) != ESP_OK) {
            continue;
        }
        video_operation_video_frame(video_fd);
        video_free_video_frame(video_fd);
        if (app_camera_video.video_task_delete) {
            app_camera_video.video_task_delete = false;
            video_stream_stop(video_fd);
            xSemaphoreGive(app_camera_video.video_stop_sem);
            vTaskDelete(NULL);
        }
    }
}

esp_err_t app_video_stream_task_start(int video_fd, int core_id)
{
    if (video_stream_start(video_fd) != ESP_OK) {
        return ESP_FAIL;
    }
    BaseType_t r = xTaskCreatePinnedToCore(video_stream_task, "video stream", VIDEO_TASK_STACK_SIZE, &video_fd, VIDEO_TASK_PRIORITY, &app_camera_video.video_stream_task_handle, core_id);
    if (r != pdPASS) {
        video_stream_stop(video_fd);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t app_video_stream_task_stop(int video_fd)
{
    app_camera_video.video_task_delete = true;
    return ESP_OK;
}

esp_err_t app_video_wait_video_stop(void)
{
    return xSemaphoreTake(app_camera_video.video_stop_sem, portMAX_DELAY);
}

esp_err_t app_video_register_frame_operation_cb(app_video_frame_operation_cb_t operation_cb)
{
    app_camera_video.user_camera_video_frame_operation_cb = operation_cb;
    return ESP_OK;
}

esp_err_t app_video_close(int video_fd)
{
    close(video_fd);
    return ESP_OK;
}

// --- V4L2 controls helpers ---

esp_err_t app_video_set_control(int video_fd, uint32_t id, int32_t value)
{
    struct v4l2_control ctrl = { .id = id, .value = value };
    if (ioctl(video_fd, VIDIOC_S_CTRL, &ctrl) != 0) {
        ESP_LOGW(TAG, "VIDIOC_S_CTRL id=0x%x val=%d failed (errno=%d)", id, value, errno);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t app_video_get_control(int video_fd, uint32_t id, int32_t *out_value)
{
    if (!out_value) return ESP_ERR_INVALID_ARG;
    struct v4l2_control ctrl = { .id = id };
    if (ioctl(video_fd, VIDIOC_G_CTRL, &ctrl) != 0) {
        ESP_LOGW(TAG, "VIDIOC_G_CTRL id=0x%x failed (errno=%d)", id, errno);
        return ESP_FAIL;
    }
    *out_value = ctrl.value;
    return ESP_OK;
}

esp_err_t app_video_set_control_normalized(int video_fd, uint32_t id, float norm_value)
{
    if (norm_value < 0.0f) norm_value = 0.0f;
    if (norm_value > 1.0f) norm_value = 1.0f;
    struct v4l2_queryctrl q = { .id = id };
    if (ioctl(video_fd, VIDIOC_QUERYCTRL, &q) != 0) {
        ESP_LOGD(TAG, "VIDIOC_QUERYCTRL id=0x%x failed (errno=%d)", id, errno);
        return ESP_FAIL;
    }
    if (q.flags & V4L2_CTRL_FLAG_DISABLED) {
        ESP_LOGD(TAG, "V4L2 ctrl 0x%x disabled", id);
        return ESP_FAIL;
    }
    long minv = q.minimum;
    long maxv = q.maximum;
    long step = q.step ? q.step : 1;
    long target = (long)(minv + norm_value * (float)(maxv - minv));
    // Align to step
    if (step > 1) {
        long off = (target - minv) % step;
        target -= off;
    }
    return app_video_set_control(video_fd, id, (int32_t)target);
}

bool app_video_has_control(int video_fd, uint32_t id)
{
    struct v4l2_queryctrl q = { .id = id };
    if (ioctl(video_fd, VIDIOC_QUERYCTRL, &q) != 0) return false;
    if (q.flags & V4L2_CTRL_FLAG_DISABLED) return false;
    return true;
}

void app_video_apply_pose_tuning(int video_fd)
{
    // Optimized ISP settings for pose detection
    // Tuned to enhance keypoint visibility and edge definition

    // Contrast: enhanced to help model distinguish body parts
    if (app_video_has_control(video_fd, V4L2_CID_CONTRAST)) {
        (void)app_video_set_control_normalized(video_fd, V4L2_CID_CONTRAST, 0.60f);
        ESP_LOGI("app_video", "Set contrast to 0.60 for better pose detection");
    }

    // Sharpness: enhance edges for better keypoint detection
    if (app_video_has_control(video_fd, V4L2_CID_SHARPNESS)) {
        (void)app_video_set_control_normalized(video_fd, V4L2_CID_SHARPNESS, 0.65f);
        ESP_LOGI("app_video", "Set sharpness to 0.65 for better edge detection");
    }

    // Saturation: moderate boost to help distinguish skin/clothing
    if (app_video_has_control(video_fd, V4L2_CID_SATURATION)) {
        (void)app_video_set_control_normalized(video_fd, V4L2_CID_SATURATION, 0.55f);
        ESP_LOGI("app_video", "Set saturation to 0.55");
    }

    // Gamma: lower value for better shadow detail
    if (app_video_has_control(video_fd, V4L2_CID_GAMMA)) {
        (void)app_video_set_control_normalized(video_fd, V4L2_CID_GAMMA, 0.45f);
        ESP_LOGI("app_video", "Set gamma to 0.45 for shadow detail");
    }

    // Ensure auto exposure is enabled
#ifdef V4L2_CID_EXPOSURE_AUTO
    if (app_video_has_control(video_fd, V4L2_CID_EXPOSURE_AUTO)) {
        (void)app_video_set_control(video_fd, V4L2_CID_EXPOSURE_AUTO, 1);
    }
#endif
}
