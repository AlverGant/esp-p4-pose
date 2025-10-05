#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_memory_utils.h"
#include "esp_private/esp_cache_private.h"
#include "esp_cache.h"

#include "bsp/esp-bsp.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_cam_sensor_xclk.h"

#include "app_video.h"
#include "app_video_utils.h"
#include "pose_overlay.h"
#include "net_telegram.h"
#include "coproc_uart.h"
// #include "coproc_sdio.h"  // SDIO disabled - using UART only
#include "fall_notifier.h"
#include "c6_flash_bridge.h"
#include "driver/uart.h"

static const char *TAG = "app_main";

#define FALL_DETECTION_LED_GPIO 23  // GPIO do LED no ESP32-P4-EYE (flashlight)
// Resolution used for pose inference (decoupled from LCD resolution)
// Using 960x960 to match camera crop size (no additional scaling needed)
// This eliminates one PPA scaling step and improves quality!
// Expected inference time: ~6s on ESP32-P4 (vs ~3s for 640x640)
#define POSE_INPUT_RES 960

static esp_lcd_panel_handle_t s_panel = NULL;
static size_t s_cache_align = 128; // default, will query later
static uint8_t *s_canvas[EXAMPLE_CAM_BUF_NUM] = {0};
// Dual-buffer for pose: allows capturing new frames while inference is running
static uint8_t *s_pose_buf[2] = {NULL, NULL};
static volatile int s_pose_buf_idx = 0; // Alternates between 0 and 1
static volatile int s_last_frame_idx = 0; // Track latest frame buffer for photo capture
static SemaphoreHandle_t s_canvas_mutex = NULL; // Protect canvas buffer during photo capture
/* no LVGL path: direct panel draw */

static inline size_t align_up(size_t v, size_t a) { return (v + (a - 1)) & ~(a - 1); }

#if CONFIG_COPROC_LOG_TEST_ONLY
// Removed ping spam task; LOG_TEST_ONLY still enables UART logging and button FALL
#endif

// REMOVIDO: c6_monitor_task causava conflito com coproc_uart_rx_task
// Ambas tentavam ler do mesmo UART, causando perda de dados
// Agora apenas coproc_uart_rx_task lida com recepção
// As respostas do C6 aparecem nos logs como "C6[u2]: ..."

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
            ESP_LOGI(TAG, "Button press confirmed, enviando alerta via notifier");
            int persons = 0, age_ms = 0, seq = 0;
            pose_overlay_get_stats(&persons, &age_ms, &seq);
            esp_err_t result = fall_notifier_send_event("Botão P1 acionado!", persons, age_ms, seq, true);
            if (result == ESP_OK) {
                ESP_LOGI(TAG, "Alerta manual enviado com sucesso");
            } else {
                ESP_LOGW(TAG, "Falha ao enviar alerta manual (%s)", esp_err_to_name(result));
            }

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
    // Quick mutex check - don't block camera pipeline
    if (!s_canvas_mutex || xSemaphoreTake(s_canvas_mutex, 0) != pdTRUE) {
        // Mutex locked (photo capture in progress), skip this frame
        return;
    }

    // Track latest frame for photo capture
    s_last_frame_idx = camera_buf_index;

    // Scale & center-crop to LCD size
    size_t out_sz = align_up(BSP_LCD_H_RES * BSP_LCD_V_RES * 2, s_cache_align);

    // Crop camera to square, then scale to LCD
    // Camera: 1920x1080 → crop to 960x960 (center) → scale to 240x240 (LCD)
    // Using 960x960 crop (same as factory_demo scale_level 1) for best quality/performance
    uint32_t crop_size = 960;  // Match factory demo's proven crop size

    (void)app_image_process_scale_crop(
        camera_buf, cam_w, cam_h,
        crop_size, crop_size,  // Square crop (960x960) from center of 1920x1080
        s_canvas[camera_buf_index], BSP_LCD_H_RES, BSP_LCD_V_RES, out_sz,
        PPA_SRM_ROTATION_ANGLE_0);


    // Swap to match LCD endianness if needed
    uint16_t *be_buf = (uint16_t *)s_canvas[camera_buf_index];
    swap_rgb565_bytes(be_buf, BSP_LCD_H_RES * BSP_LCD_V_RES);

    // Build dedicated pose input at fixed center crop (no ROI/zoom)
    // Using dual-buffer: alternate between buffers to avoid blocking on inference
    if (s_pose_buf[0] && s_pose_buf[1]) {
        size_t pose_out_sz = align_up(POSE_INPUT_RES * POSE_INPUT_RES * 2, s_cache_align);

        // Alternate between the two pose buffers
        s_pose_buf_idx = (s_pose_buf_idx + 1) % 2;
        uint8_t *current_pose_buf = s_pose_buf[s_pose_buf_idx];

        // Use same 960x960 crop for pose input for consistency
        // Fixed 0° rotation (no adaptive rotation)
        (void)app_image_process_scale_crop(
            camera_buf, cam_w, cam_h,
            crop_size, crop_size,  // Uses same 960x960 crop as display
            current_pose_buf, POSE_INPUT_RES, POSE_INPUT_RES, pose_out_sz,
            PPA_SRM_ROTATION_ANGLE_0);
        // Pose expects big-endian RGB565 like LCD pipeline
        swap_rgb565_bytes((uint16_t *)current_pose_buf, POSE_INPUT_RES * POSE_INPUT_RES);

        // Submit frames to pose task on a time basis
        // Dual-buffer allows us to always submit fresh frames without blocking
        static int64_t last_submit_us = 0;
        int64_t now_us = esp_timer_get_time();
        const int64_t min_period_us = 500000; // 0.5 second
        if (now_us - last_submit_us >= min_period_us) {
            esp_err_t q = pose_overlay_submit((uint16_t *)current_pose_buf, POSE_INPUT_RES, POSE_INPUT_RES);
            if (q == ESP_OK) {
                ESP_LOGI(TAG, "Submitting pose buffer %dx%d (queued, buf=%d)",
                         POSE_INPUT_RES, POSE_INPUT_RES, s_pose_buf_idx);
            } else if (q == ESP_ERR_INVALID_STATE) {
                // Staged while inference busy - dual buffer will have fresh data next time
                ESP_LOGD(TAG, "Staged pose buffer %dx%d (busy, buf=%d)",
                         POSE_INPUT_RES, POSE_INPUT_RES, s_pose_buf_idx);
            } else {
                ESP_LOGW(TAG, "Pose submit failed (%d)", (int)q);
            }
            last_submit_us = now_us;
        }
    }
    (void)pose_overlay_draw(be_buf, BSP_LCD_H_RES, BSP_LCD_V_RES);

    // Border clearing removed - let image fill entire display
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, BSP_LCD_H_RES, BSP_LCD_V_RES, be_buf);

    // Release mutex after frame processing complete
    if (s_canvas_mutex) {
        xSemaphoreGive(s_canvas_mutex);
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

    int64_t now_us = esp_timer_get_time(); // Get current time for LED control

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
    // Draw already done above before mutex release
}

const uint16_t* app_main_get_latest_frame(int *width, int *height)
{
    // Lock mutex to prevent frame_cb from modifying buffer during photo capture
    if (s_canvas_mutex && xSemaphoreTake(s_canvas_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to lock canvas mutex for photo capture");
        if (width) *width = 0;
        if (height) *height = 0;
        return NULL;
    }

    if (!s_canvas[s_last_frame_idx]) {
        if (s_canvas_mutex) xSemaphoreGive(s_canvas_mutex);
        if (width) *width = 0;
        if (height) *height = 0;
        return NULL;
    }

    // Frame is already big-endian RGB565 after swap, ready for JPEG
    if (width) *width = BSP_LCD_H_RES;
    if (height) *height = BSP_LCD_V_RES;

    ESP_LOGI(TAG, "Locked frame %d (%dx%d) for photo capture",
             s_last_frame_idx, BSP_LCD_H_RES, BSP_LCD_V_RES);

    // NOTE: Caller MUST call app_main_release_frame() after done!
    return (const uint16_t *)s_canvas[s_last_frame_idx];
}

void app_main_release_frame(void)
{
    if (s_canvas_mutex) {
        xSemaphoreGive(s_canvas_mutex);
        ESP_LOGD(TAG, "Released frame mutex");
    }
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

    // Start co-processor UART communication with C6
#if CONFIG_COPROC_UART_ENABLE
    // STEP 1: Initialize UART FIRST (creates semaphore for C6_READY detection)
    ESP_LOGI(TAG, "Initializing UART communication with C6...");
    esp_err_t uart_init_ret = coproc_uart_init();
    if (uart_init_ret != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(uart_init_ret));
    } else {
        ESP_LOGI(TAG, "UART init successful, starting RX log task");
        // Esta é a ÚNICA task que deve ler do UART
        (void)coproc_uart_start_rx_log();
        ESP_LOGI(TAG, "RX monitoring active - ready to detect C6_READY signal");

        // Give RX task time to start and be ready
        vTaskDelay(pdMS_TO_TICKS(200));

        // STEP 2: Now reset C6 (with UART RX already listening for C6_READY)
        ESP_LOGI(TAG, "Ensuring C6 boots from flash (not download mode)...");
        gpio_reset_pin(GPIO_NUM_33);
        gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_NUM_33, 1);  // HIGH = boot from flash (not download mode)

        ESP_LOGI(TAG, "Resetting C6 coprocessor...");
        gpio_reset_pin(GPIO_NUM_9);
        gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_NUM_9, 0);  // Assert reset (LOW)
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(GPIO_NUM_9, 1);  // Release reset (HIGH)

        // STEP 3: Wait for C6_READY signal (C6 boots, connects WiFi, syncs time, then sends C6_READY)
        // This can take up to 30-40 seconds depending on WiFi/SNTP!
        ESP_LOGI(TAG, "⏳ Waiting for C6_READY signal (timeout: 45 seconds)...");
        ESP_LOGI(TAG, "   C6 needs time to: boot → WiFi connect → SNTP sync → send C6_READY");
        esp_err_t ready_ret = coproc_uart_wait_for_c6_ready(45000);  // 45s timeout

        if (ready_ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ C6 is ready! Proceeding with P4 initialization");

            // Now safe to send test message
            ESP_LOGI(TAG, "Sending test message to C6...");
            esp_err_t send_ret = coproc_uart_send_line("P4_PING");
            ESP_LOGI(TAG, "Test send result: %s (expecting C6_ACK)", esp_err_to_name(send_ret));
            vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for C6_ACK response
        } else {
            ESP_LOGW(TAG, "⚠️  C6_READY timeout! C6 may not be responding properly");
            ESP_LOGW(TAG, "    Check C6 logs for WiFi/SNTP issues. Continuing anyway...");
        }
    }

    // SDIO DISABLED: Interferes with UART communication (resets C6 during init)
    ESP_LOGI(TAG, "SDIO disabled - using UART only for C6 communication");
    // esp_err_t sdio_ret = coproc_sdio_init();
    // if (sdio_ret == ESP_OK) {
    //     (void)coproc_sdio_start_rx_log();
    //     ESP_LOGI(TAG, "SDIO communication initialized (UART remains active)");
    // } else {
    //     ESP_LOGW(TAG, "SDIO init failed, using UART only");
    // }
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

    // TESTE: BSP minimalista - power rails SEM XCLK
    ESP_LOGW(TAG, "🔧 TESTE #2: BSP sem XCLK (apenas power rails)");
    ESP_LOGW(TAG, "🔧 XCLK 24MHz desabilitado para evitar interferência UART");

    // Network: WiFi P4 permanece DESABILITADO (usuário não quer)
    ESP_LOGW(TAG, "WIFI_SSID vazio – pulando Wi‑Fi P4");

    // BSP CUSTOMIZADO: Init completo EXCETO C6_EN_PIN
    // IMPORTANTE: rtc_gpio_init(BSP_C6_EN_PIN) reseta o C6, quebrando comunicação UART
    // Solução: Inicializar todos os componentes do BSP, mas pular C6_EN_PIN
    ESP_LOGI(TAG, "Init BSP (custom - skip C6_EN_PIN to avoid C6 reset)");
    {
        // XCLK 24MHz para câmera
        esp_cam_sensor_xclk_config_t cam_xclk_config = {
            .esp_clock_router_cfg = {
                .xclk_pin = BSP_CAMERA_XCLK_PIN,
                .xclk_freq_hz = BSP_MIPI_CAMERA_XCLK_FREQUENCY,
            }
        };
        esp_cam_sensor_xclk_handle_t xclk_handle = NULL;
        ESP_ERROR_CHECK(esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &xclk_handle));
        ESP_ERROR_CHECK(esp_cam_sensor_xclk_start(xclk_handle, &cam_xclk_config));

        // SD card power enable (GPIO normal)
        const gpio_config_t sdcard_io_config = {
            .pin_bit_mask = BIT64(BSP_SD_EN_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&sdcard_io_config));
        gpio_set_level(BSP_SD_EN_PIN, 0);  // SD desabilitado

        // Camera reset pin (GPIO normal)
        const gpio_config_t rst_io_config = {
            .pin_bit_mask = BIT64(BSP_CAMERA_RST_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&rst_io_config));
        gpio_set_level(BSP_CAMERA_RST_PIN, 1);  // RST high

        // Sleep power domain config
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
        esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_ON);

        // RTC GPIO APENAS para camera, NÃO para C6!
        rtc_gpio_init(BSP_CAMERA_EN_PIN);
        // CRÍTICO: NÃO chamar rtc_gpio_init(BSP_C6_EN_PIN) - reseta o C6!
        rtc_gpio_set_direction(BSP_CAMERA_EN_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
        rtc_gpio_pulldown_dis(BSP_CAMERA_EN_PIN);
        rtc_gpio_pullup_dis(BSP_CAMERA_EN_PIN);
        rtc_gpio_hold_dis(BSP_CAMERA_EN_PIN);
        rtc_gpio_set_level(BSP_CAMERA_EN_PIN, 1);  // Camera power ON
        rtc_gpio_hold_en(BSP_CAMERA_EN_PIN);

        ESP_LOGI(TAG, "✅ Custom BSP initialized (C6_EN_PIN skipped)");
    }

    // I2C init (necessário para câmera SCCB)
    ESP_LOGI(TAG, "Init I2C");
    ESP_ERROR_CHECK(bsp_i2c_init());
    i2c_master_bus_handle_t i2c_handle;
    ESP_ERROR_CHECK(bsp_get_i2c_bus_handle(&i2c_handle));

    // Display init
    ESP_LOGI(TAG, "Init display");
    esp_lcd_panel_io_handle_t io = NULL;
    const bsp_display_config_t disp_cfg = {
        .max_transfer_sz = BSP_LCD_H_RES * 10 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(bsp_display_new(&disp_cfg, &s_panel, &io));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    ESP_ERROR_CHECK(bsp_display_backlight_on());

    // Create canvas mutex for photo capture protection
    s_canvas_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_canvas_mutex ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_LOGI(TAG, "Created canvas mutex for photo capture");

    // PPA (Pixel Processing Accelerator)
    ESP_LOGI(TAG, "Init PPA utils");
    ESP_ERROR_CHECK(app_video_utils_init());

    // Pose overlay (ESP-DL)
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

    // Camera MIPI-CSI
    ESP_LOGI(TAG, "Init camera");
    (void)esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &s_cache_align);
    size_t canvas_sz = align_up(BSP_LCD_H_RES * BSP_LCD_V_RES * 2, s_cache_align);
    for (int i = 0; i < EXAMPLE_CAM_BUF_NUM; ++i) {
        s_canvas[i] = heap_caps_aligned_calloc(s_cache_align, 1, canvas_sz, MALLOC_CAP_SPIRAM);
        ESP_ERROR_CHECK(s_canvas[i] ? ESP_OK : ESP_ERR_NO_MEM);
    }
    // Allocate dual pose buffers for non-blocking frame capture during inference
    size_t pose_sz = align_up(POSE_INPUT_RES * POSE_INPUT_RES * 2, s_cache_align);
    for (int i = 0; i < 2; i++) {
        s_pose_buf[i] = heap_caps_aligned_calloc(s_cache_align, 1, pose_sz, MALLOC_CAP_SPIRAM);
        ESP_ERROR_CHECK(s_pose_buf[i] ? ESP_OK : ESP_ERR_NO_MEM);
    }
    ESP_LOGI(TAG, "Allocated dual pose buffers %dx%d (%u bytes each)",
             POSE_INPUT_RES, POSE_INPUT_RES, (unsigned)pose_sz);
    ESP_ERROR_CHECK(app_video_main(i2c_handle));
    int cam_fd = app_video_open(EXAMPLE_CAM_DEV_PATH, APP_VIDEO_FMT);
    ESP_ERROR_CHECK_WITHOUT_ABORT((cam_fd >= 0) ? ESP_OK : ESP_FAIL);
    if (cam_fd < 0) {
        ESP_LOGE(TAG, "Failed to open camera");
        return;
    }
    // Minimal tuning: only slight contrast boost for better pose detection
    app_video_apply_pose_tuning(cam_fd);
    ESP_ERROR_CHECK(app_video_set_bufs(cam_fd, EXAMPLE_CAM_BUF_NUM, NULL));
    ESP_ERROR_CHECK(app_video_register_frame_operation_cb(frame_cb));
    ESP_LOGI(TAG, "Start video stream");
    ESP_ERROR_CHECK(app_video_stream_task_start(cam_fd, 0));
}
