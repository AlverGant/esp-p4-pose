#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_memory_utils.h"
#include "esp_private/esp_cache_private.h"

#include "bsp/esp-bsp.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "app_video.h"
#include "app_video_utils.h"
#include "pose_overlay.h"
#include "net_telegram.h"
#include "coproc_uart.h"
#include "coproc_sdio.h"
#include "fall_notifier.h"
#include "c6_flash_bridge.h"
#include "driver/uart.h"

static const char *TAG = "app_main";

#define FALL_DETECTION_LED_GPIO 23  // GPIO do LED no ESP32-P4-EYE (flashlight)
// Resolution used for pose inference (decoupled from LCD resolution)
// Tip: multiples of 32 tend to work best with s8 models. 416 gives better
// detail at distance than 320, at the cost of more latency.
#define POSE_INPUT_RES 640

static esp_lcd_panel_handle_t s_panel = NULL;
static size_t s_cache_align = 128; // default, will query later
static uint8_t *s_canvas[EXAMPLE_CAM_BUF_NUM] = {0};
static uint8_t *s_pose_buf = NULL; // RGB565 buffer (POSE_INPUT_RES x POSE_INPUT_RES)
/* no LVGL path: direct panel draw */

static inline size_t align_up(size_t v, size_t a) { return (v + (a - 1)) & ~(a - 1); }

#if CONFIG_COPROC_LOG_TEST_ONLY
// Removed ping spam task; LOG_TEST_ONLY still enables UART logging and button FALL
#endif

// Task para monitorar respostas do C6
static void c6_monitor_task(void *arg)
{
    (void)arg;
    const int uart_port = CONFIG_COPROC_UART_NUM; // Mesmo UART usado para comunicação
    uint8_t buf[128];

    ESP_LOGI(TAG, "C6 monitor task started - listening on UART%d", uart_port);

    // Certifica que o driver já está instalado (feito pelo coproc_uart)
    vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda inicialização

    // Verificar se driver está instalado
    if (!uart_is_driver_installed(uart_port)) {
        ESP_LOGE(TAG, "UART%d driver not installed for C6 monitor!", uart_port);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "C6 monitor: UART%d driver confirmed, starting monitor loop", uart_port);

    for (;;) {
        int len = uart_read_bytes(uart_port, buf, sizeof(buf) - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            buf[len] = '\0';
            ESP_LOGI(TAG, "C6_RESPONSE: %s", buf); // Mostrar respostas do C6
        }
    }
}

// Button on GPIO3 triggers a manual FALL test message to C6
#define FALL_TEST_BTN_GPIO 3
static void fall_test_btn_task(void *arg)
{
    (void)arg;
    int last = 1;
    static int debug_counter = 0;

    ESP_LOGI(TAG, "Fall test button task started on GPIO%d", FALL_TEST_BTN_GPIO);

    for (;;) {
        int cur = gpio_get_level(FALL_TEST_BTN_GPIO);

        // Debug log a cada 100 iterações para confirmar que a task está rodando
        if (++debug_counter >= 100) {
            ESP_LOGD(TAG, "Button task alive, GPIO%d level=%d", FALL_TEST_BTN_GPIO, cur);
            debug_counter = 0;
        }

        if (last == 1 && cur == 0) { // pressed (assuming pull-up, active low)
            ESP_LOGI(TAG, "Button GPIO%d pressed!", FALL_TEST_BTN_GPIO);
            // simple debounce
            vTaskDelay(pdMS_TO_TICKS(80));
            if (gpio_get_level(FALL_TEST_BTN_GPIO) == 0) {
                ESP_LOGI(TAG, "Button press confirmed, sending FALL message");
                char line[96];
#ifdef CONFIG_COPROC_MSG_TEMPLATE
                snprintf(line, sizeof(line), CONFIG_COPROC_MSG_TEMPLATE, 1, 0, 999);
#else
                snprintf(line, sizeof(line), "FALL persons=%d age_ms=%d seq=%d", 1, 0, 999);
#endif
                ESP_LOGI(TAG, "Sending: %s", line);
                // Try UART first, fallback to SDIO
                esp_err_t result = coproc_uart_send_line(line);
                if (result != ESP_OK) {
                    ESP_LOGW(TAG, "UART send failed, trying SDIO");
                    result = coproc_sdio_send_line(line);
                }
                ESP_LOGI(TAG, "Send result: %s", esp_err_to_name(result));

                // wait release
                while (gpio_get_level(FALL_TEST_BTN_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                ESP_LOGI(TAG, "Button released");
            }
        }
        last = cur;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

static void frame_cb(uint8_t *camera_buf, uint8_t camera_buf_index, uint32_t cam_w, uint32_t cam_h, size_t cam_len)
{
    // Scale & center-crop to LCD size
    size_t out_sz = align_up(BSP_LCD_H_RES * BSP_LCD_V_RES * 2, s_cache_align);

    // Clear entire canvas with solid black background (ensure complete clearing)
    memset(s_canvas[camera_buf_index], 0, out_sz);

    // Additional clearing for potential padding/alignment issues
    size_t total_buffer_size = align_up(BSP_LCD_H_RES * BSP_LCD_V_RES * 2, s_cache_align);
    if (total_buffer_size > out_sz) {
        // Clear any additional aligned bytes
        memset((uint8_t*)s_canvas[camera_buf_index] + out_sz, 0, total_buffer_size - out_sz);
    }

    // Crop camera to square maintaining aspect ratio, then scale to fill LCD
    // Camera: 1280x720 (16:9) → crop to 720x720 (1:1) → scale to 480x480
    uint32_t crop_size = cam_h; // Use camera height (720) as crop size - safe value

    (void)app_image_process_scale_crop(
        camera_buf, cam_w, cam_h,
        crop_size, crop_size,  // Square crop (720x720) from center of 1280x720
        s_canvas[camera_buf_index], BSP_LCD_H_RES, BSP_LCD_V_RES, out_sz,
        PPA_SRM_ROTATION_ANGLE_0);

    // Swap to match LCD endianness if needed
    uint16_t *be_buf = (uint16_t *)s_canvas[camera_buf_index];
    swap_rgb565_bytes(be_buf, BSP_LCD_H_RES * BSP_LCD_V_RES);

    // Build dedicated pose input at fixed center crop (no ROI/zoom)
    if (s_pose_buf) {
        size_t pose_out_sz = align_up(POSE_INPUT_RES * POSE_INPUT_RES * 2, s_cache_align);
        static ppa_srm_rotation_angle_t s_pose_rot = PPA_SRM_ROTATION_ANGLE_0;
        // Adaptive rotation: if seguidas inferências retornarem 0 pessoas, tente outra rotação
        static int last_seq = -1;
        static int zero_streak = 0;
        int persons = 0, age_ms = 0, seq = -1;
        pose_overlay_get_stats(&persons, &age_ms, &seq);
        if (seq != last_seq && seq >= 0) {
            if (persons <= 0) {
                zero_streak++;
            } else {
                // If persons detected, revert to 0 deg after one confirmation
                zero_streak = 0;
                if (s_pose_rot != PPA_SRM_ROTATION_ANGLE_0) {
                    s_pose_rot = PPA_SRM_ROTATION_ANGLE_0;
                    pose_overlay_set_input_rotation(0);
                    ESP_LOGI(TAG, "Persons detected; reverting pose input rotation to 0 deg");
                }
            }
            last_seq = seq;
            if (zero_streak >= 2) {
                // Toggle only between 0 and 180 degrees
                s_pose_rot = (s_pose_rot == PPA_SRM_ROTATION_ANGLE_0) ? PPA_SRM_ROTATION_ANGLE_180 : PPA_SRM_ROTATION_ANGLE_0;
                int deg = (s_pose_rot == PPA_SRM_ROTATION_ANGLE_0) ? 0 : 180;
                pose_overlay_set_input_rotation(deg);
                ESP_LOGW(TAG, "No persons detected; rotating pose input to %d deg", deg);
                zero_streak = 0;
            }
        }
        (void)app_image_process_scale_crop(
            camera_buf, cam_w, cam_h,
            crop_size, crop_size,
            s_pose_buf, POSE_INPUT_RES, POSE_INPUT_RES, pose_out_sz,
            s_pose_rot);
        // Pose expects big-endian RGB565 like LCD pipeline
        swap_rgb565_bytes((uint16_t *)s_pose_buf, POSE_INPUT_RES * POSE_INPUT_RES);
    }

    // Submit frames to pose task on a time basis (not frame-count)
    // v2 model has ~3s inference; we queue a fresh frame at ~1 Hz so
    // the pose task always has a recent image without waiting for 45 frames.
    static int64_t last_submit_us = 0;
    int64_t now_us = esp_timer_get_time();
    const int64_t min_period_us = 500000; // 0.5 second
    if (now_us - last_submit_us >= min_period_us && s_pose_buf) {
        esp_err_t q = pose_overlay_submit((uint16_t *)s_pose_buf, POSE_INPUT_RES, POSE_INPUT_RES);
        if (q == ESP_OK) {
            ESP_LOGI(TAG, "Submitting pose buffer %dx%d (queued)", POSE_INPUT_RES, POSE_INPUT_RES);
        } else if (q == ESP_ERR_INVALID_STATE) {
            // Staged while inference busy; avoid spamming logs
            // ESP_LOGD(TAG, "Staged pose buffer %dx%d (busy)", POSE_INPUT_RES, POSE_INPUT_RES);
        } else {
            ESP_LOGW(TAG, "Pose submit failed (%d)", (int)q);
        }
        last_submit_us = now_us;
    }
    (void)pose_overlay_draw(be_buf, BSP_LCD_H_RES, BSP_LCD_V_RES);

    // Clean display borders even more thoroughly
    // Clear bottom 16 pixels completely
    for (int y = BSP_LCD_V_RES - 16; y < BSP_LCD_V_RES; y++) {
        for (int x = 0; x < BSP_LCD_H_RES; x++) {
            be_buf[y * BSP_LCD_H_RES + x] = 0;
        }
    }
    // Clear right 16 pixels completely
    for (int x = BSP_LCD_H_RES - 16; x < BSP_LCD_H_RES; x++) {
        for (int y = 0; y < BSP_LCD_V_RES; y++) {
            be_buf[y * BSP_LCD_H_RES + x] = 0;
        }
    }

    // LED control:
    // - Steady ON when fall detected (latched inside pose overlay)
    // - Blink when persons detected but no fall
    // - OFF otherwise
    static bool led_state = false;
    static int64_t last_blink_us = 0;
    const int64_t blink_period_us = 600000; // ~0.6s period (~1.7Hz)

    bool fall = pose_overlay_is_fall_detected();
    int persons = 0, age_ms = 0, seq = 0;
    pose_overlay_get_stats(&persons, &age_ms, &seq);

    if (fall) {
        if (!led_state) {
            gpio_set_level(FALL_DETECTION_LED_GPIO, 1); // LED ON steady
            led_state = true;
            ESP_LOGI(TAG, "FALL DETECTED - LED ON");
        }
        // keep LED on; do not blink
        last_blink_us = now_us; // reset blink timer
    } else if (persons > 0 && age_ms >= 0 && age_ms <= 8000) {
        // Blink when recent inference has people
        if (now_us - last_blink_us >= (blink_period_us / 2)) {
            led_state = !led_state;
            gpio_set_level(FALL_DETECTION_LED_GPIO, led_state ? 1 : 0);
            last_blink_us = now_us;
        }
    } else {
        if (led_state) {
            gpio_set_level(FALL_DETECTION_LED_GPIO, 0); // LED OFF
            led_state = false;
            ESP_LOGI(TAG, "No people - LED OFF");
        }
        last_blink_us = now_us;
    }

    // Draw full screen (end coords exclusive) — baseline working path
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, BSP_LCD_H_RES, BSP_LCD_V_RES, s_canvas[camera_buf_index]);
}

void app_main(void)
{
    // If bridge trigger held at boot, start USB<->UART bridge for C6 flashing
    if (c6_flash_bridge_try_start()) {
        return; // never returns
    }

    ESP_LOGI(TAG, "Init NVS");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Start co-processor SDIO communication with C6
#if CONFIG_COPROC_UART_ENABLE
    // Always initialize UART for fallback communication
    ESP_LOGI(TAG, "Initializing UART communication with C6...");
    (void)coproc_uart_init();
    (void)coproc_uart_start_rx_log();

    // Also try SDIO (but use UART as primary for now due to SDIO issues)
    ESP_LOGI(TAG, "Attempting SDIO communication with C6...");
    esp_err_t sdio_ret = coproc_sdio_init();
    if (sdio_ret == ESP_OK) {
        (void)coproc_sdio_start_rx_log();
        ESP_LOGI(TAG, "SDIO communication initialized (UART remains active)");
    } else {
        ESP_LOGW(TAG, "SDIO init failed, using UART only");
    }
#endif

#if CONFIG_COPROC_LOG_TEST_ONLY
    ESP_LOGW(TAG, "COPROC_LOG_TEST_ONLY enabled: skipping display/camera; logging C6 UART only");
    // Configure button and start FALL test task as well
    {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << FALL_TEST_BTN_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        xTaskCreatePinnedToCore(fall_test_btn_task, "fall_btn", 4096, NULL, 1, NULL, 0);
    }
    return;
#endif

    // Network: connect Wi‑Fi if configured
    if (strlen(CONFIG_WIFI_SSID) > 0) {
        ESP_LOGI(TAG, "Init Wi‑Fi");
        (void)net_init_wifi();
    } else {
        ESP_LOGW(TAG, "WIFI_SSID vazio – pulando Wi‑Fi");
    }

    // Board init (power rails, XCLK, etc.)
    ESP_LOGI(TAG, "Init BSP");
    ESP_ERROR_CHECK(bsp_p4_eye_init());

    // Display: bring up panel via BSP at 80 MHz (original working path)
    ESP_LOGI(TAG, "Init display (BSP 80MHz)");
    esp_lcd_panel_io_handle_t io = NULL;
    const bsp_display_config_t disp_cfg = {
        .max_transfer_sz = BSP_LCD_H_RES * 10 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(bsp_display_new(&disp_cfg, &s_panel, &io));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    ESP_ERROR_CHECK(bsp_display_backlight_on());

    // I2C for SCCB
    ESP_LOGI(TAG, "Init I2C");
    ESP_ERROR_CHECK(bsp_i2c_init());
    i2c_master_bus_handle_t i2c_handle;
    ESP_ERROR_CHECK(bsp_get_i2c_bus_handle(&i2c_handle));

    // PPA utils
    ESP_LOGI(TAG, "Init PPA utils");
    ESP_ERROR_CHECK(app_video_utils_init());

    // Pose overlay init (ESP-DL)
    ESP_LOGI(TAG, "Init Pose overlay");
    ESP_ERROR_CHECK(pose_overlay_init());

    // Button GPIO (also enable manual FALL test in full app)
    {
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << FALL_TEST_BTN_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io);
        xTaskCreatePinnedToCore(fall_test_btn_task, "fall_btn", 4096, NULL, 1, NULL, 0);
    }

    // Start fall notifier (Telegram)
    start_fall_notifier_task();

    // RX log já é feito por coproc_uart_start_rx_log(); evitar dois leitores no mesmo UART

    // Init LED GPIO for fall detection
    ESP_LOGI(TAG, "Init Fall Detection LED");
    ESP_ERROR_CHECK(gpio_set_direction(FALL_DETECTION_LED_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(FALL_DETECTION_LED_GPIO, 0)); // Start with LED OFF

    // Cache alignment for DMA-friendly buffers
    (void)esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &s_cache_align);

    // Allocate two canvas buffers for LCD blit
    size_t canvas_sz = align_up(BSP_LCD_H_RES * BSP_LCD_V_RES * 2, s_cache_align);
    for (int i = 0; i < EXAMPLE_CAM_BUF_NUM; ++i) {
        s_canvas[i] = heap_caps_aligned_calloc(s_cache_align, 1, canvas_sz, MALLOC_CAP_SPIRAM);
        ESP_ERROR_CHECK(s_canvas[i] ? ESP_OK : ESP_ERR_NO_MEM);
    }

    // Allocate dedicated pose buffer (higher resolution than LCD if desired)
    size_t pose_sz = align_up(POSE_INPUT_RES * POSE_INPUT_RES * 2, s_cache_align);
    s_pose_buf = heap_caps_aligned_calloc(s_cache_align, 1, pose_sz, MALLOC_CAP_SPIRAM);
    ESP_ERROR_CHECK(s_pose_buf ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_LOGI(TAG, "Allocated pose buffer %dx%d (%u bytes)", POSE_INPUT_RES, POSE_INPUT_RES, (unsigned)pose_sz);

    // No edge buffers (drawing full screen only)

    // Init camera (esp_video)
    ESP_LOGI(TAG, "Init camera");
    ESP_ERROR_CHECK(app_video_main(i2c_handle));

    int cam_fd = app_video_open(EXAMPLE_CAM_DEV_PATH, APP_VIDEO_FMT);
    ESP_ERROR_CHECK_WITHOUT_ABORT((cam_fd >= 0) ? ESP_OK : ESP_FAIL);
    if (cam_fd < 0) {
        ESP_LOGE(TAG, "Failed to open camera");
        return;
    }

    // Apply camera tuning to increase contrast/visibility for pose
    app_video_apply_pose_tuning(cam_fd);

    ESP_ERROR_CHECK(app_video_set_bufs(cam_fd, EXAMPLE_CAM_BUF_NUM, NULL));
    ESP_ERROR_CHECK(app_video_register_frame_operation_cb(frame_cb));

    // Start streaming on core 0
    ESP_LOGI(TAG, "Start video stream");
    ESP_ERROR_CHECK(app_video_stream_task_start(cam_fd, 0));
}
