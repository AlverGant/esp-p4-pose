#include "c6_flash_bridge.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "bsp/esp-bsp.h"
#include "esp_lcd_panel_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG_BR = "c6_bridge";
static esp_lcd_panel_handle_t s_panel = NULL;

static inline uint16_t be16(uint16_t v) { return (v >> 8) | (v << 8); }

#if CONFIG_C6_BRIDGE_ENABLE

// 5x7 font for ASCII 32..90 (subset)
static const uint8_t font5x7[][5] = {
    // SP !  "  #  $  %  &  '  (  )  *  +  ,  -  .  /
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5f,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7f,0x14,0x7f,0x14},
    {0x24,0x2a,0x7f,0x2a,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},
    {0x00,0x1c,0x22,0x41,0x00},{0x00,0x41,0x22,0x1c,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    // 0-9
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
    {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
    // : ; < = > ? @
    {0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
    // A-Z (65..90)
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x7F,0x20,0x18,0x20,0x7F},{0x63,0x14,0x08,0x14,0x63},{0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43}
};

static void draw_text_big(uint16_t *fb, int w, int h, int x, int y, const char *txt, int scale, uint16_t color_be)
{
    for (const char *p = txt; *p; ++p) {
        unsigned char c = (unsigned char)*p;
        if (c < 32 || c > 90) { x += 6 * scale; continue; }
        const uint8_t *bm = font5x7[c - 32];
        for (int col = 0; col < 5; ++col) {
            for (int row = 0; row < 7; ++row) {
                if ((bm[col] >> row) & 1) {
                    for (int yy = 0; yy < scale; ++yy)
                        for (int xx = 0; xx < scale; ++xx) {
                            int X = x + col*scale + xx;
                            int Y = y + row*scale + yy;
                            if ((unsigned)X < (unsigned)w && (unsigned)Y < (unsigned)h)
                                fb[Y*w + X] = color_be;
                        }
                }
            }
        }
        x += 6 * scale; // 5 + 1 space
    }
}

static void bridge_draw_screen(void)
{
    // Minimal BSP init to use LCD
    if (!s_panel) {
        bsp_p4_eye_init();
        esp_lcd_panel_io_handle_t io = NULL;
        const bsp_display_config_t disp_cfg = {
            .max_transfer_sz = BSP_LCD_H_RES * 10 * sizeof(uint16_t),
        };
        bsp_display_new(&disp_cfg, &s_panel, &io);
        esp_lcd_panel_disp_on_off(s_panel, true);
        bsp_display_backlight_on();
    }
    size_t pixels = BSP_LCD_H_RES * BSP_LCD_V_RES;
    uint16_t *fb = heap_caps_calloc(pixels, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (!fb) return;
    // Background dark
    uint16_t bg = be16(0x0841); // dark gray
    for (size_t i = 0; i < pixels; ++i) fb[i] = bg;
    // Title
    draw_text_big(fb, BSP_LCD_H_RES, BSP_LCD_V_RES, 20, 40, "C6 FLASH BRIDGE", 6, be16(0xFFFF));
    draw_text_big(fb, BSP_LCD_H_RES, BSP_LCD_V_RES, 20, 120, "USB <-> UART1", 4, be16(0xFFE0));
    draw_text_big(fb, BSP_LCD_H_RES, BSP_LCD_V_RES, 20, 160, "115200 BAUD", 4, be16(0x07FF));
    draw_text_big(fb, BSP_LCD_H_RES, BSP_LCD_V_RES, 20, 200, "BTN2 2S: RE-TRIGGER", 3, be16(0xF81F));
    draw_text_big(fb, BSP_LCD_H_RES, BSP_LCD_V_RES, 20, 235, "esptool --before no_reset", 2, be16(0xFFFF));
    draw_text_big(fb, BSP_LCD_H_RES, BSP_LCD_V_RES, 20, 255, "--after no_reset", 2, be16(0xFFFF));
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0, BSP_LCD_H_RES, BSP_LCD_V_RES, fb);
    heap_caps_free(fb);
}

static void put_c6_download_mode(void)
{
    // Hold BOOT low, pulse EN
    gpio_set_direction(CONFIG_C6_BOOT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_C6_BOOT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_direction(CONFIG_C6_EN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_C6_EN_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(CONFIG_C6_EN_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    // Keep BOOT low briefly then release
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(CONFIG_C6_BOOT_GPIO, 1); // release
}

static void bridge_task(void *arg)
{
    (void)arg;
    // Install USB Serial/JTAG driver
    usb_serial_jtag_driver_config_t usj_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&usj_cfg);

    // UART to C6
    uart_config_t uc = {
        .baud_rate = CONFIG_C6_BRIDGE_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(CONFIG_C6_BRIDGE_UART_NUM, &uc);
    uart_set_pin(CONFIG_C6_BRIDGE_UART_NUM, CONFIG_C6_BRIDGE_UART_TX, CONFIG_C6_BRIDGE_UART_RX,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(CONFIG_C6_BRIDGE_UART_NUM, 2048, 2048, 0, NULL, 0);

    // Put C6 into download mode now (retry a few times) and show LCD banner
    for (int i = 0; i < 3; ++i) {
        put_c6_download_mode();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    bridge_draw_screen();
    ESP_LOGW(TAG_BR, "Bridge active: USB <-> UART%d @%d", CONFIG_C6_BRIDGE_UART_NUM, CONFIG_C6_BRIDGE_BAUD);
    ESP_LOGW(TAG_BR, "Use esptool.py --chip esp32c6 --before no_reset --after no_reset -p <ACM0> flash");

    uint8_t buf_usj[256];
    uint8_t buf_uart[256];
    // Simple detector for esptool sync pattern 07 07 12 20
    uint8_t sync_win[4] = {0};

    // Configure re-trigger button (long press)
    gpio_config_t rt = {
        .pin_bit_mask = 1ULL << CONFIG_C6_BRIDGE_RETRIGGER_BTN_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&rt);
    int hold_ticks_needed = pdMS_TO_TICKS(CONFIG_C6_BRIDGE_RETRIGGER_HOLD_MS);
    int hold_ticks = 0;

    bool any_rx = false;
    int retry_ticks = 0;
    for (;;) {
        // USB -> UART
        int n = usb_serial_jtag_read_bytes(buf_usj, sizeof(buf_usj), 0);
        if (n > 0) {
            for (int i = 0; i < n; ++i) {
                sync_win[0] = sync_win[1];
                sync_win[1] = sync_win[2];
                sync_win[2] = sync_win[3];
                sync_win[3] = buf_usj[i];
                if (sync_win[0]==0x07 && sync_win[1]==0x07 && sync_win[2]==0x12 && sync_win[3]==0x20) {
                    // Host is attempting esptool sync; ensure C6 is in download mode
                    put_c6_download_mode();
                }
            }
            uart_write_bytes(CONFIG_C6_BRIDGE_UART_NUM, (const char *)buf_usj, n);
        }
        // UART -> USB
        int m = uart_read_bytes(CONFIG_C6_BRIDGE_UART_NUM, buf_uart, sizeof(buf_uart), 0);
        if (m > 0) {
            usb_serial_jtag_write_bytes(buf_uart, m, 0);
            any_rx = true;
        }
        // Re-trigger download if button held long enough
        if (gpio_get_level(CONFIG_C6_BRIDGE_RETRIGGER_BTN_GPIO) == 0) {
            if (hold_ticks < hold_ticks_needed) hold_ticks++;
            if (hold_ticks == hold_ticks_needed) {
                ESP_LOGW(TAG_BR, "Re-trigger: forcing C6 download mode");
                put_c6_download_mode();
            }
        } else {
            hold_ticks = 0;
        }

        // If não recebemos nada ainda, force download periodicamente (~1s)
        if (!any_rx) {
            retry_ticks += 1;
            if (retry_ticks >= pdMS_TO_TICKS(1000)) {
                retry_ticks = 0;
                ESP_LOGW(TAG_BR, "Auto-retry: forcing C6 download mode");
                put_c6_download_mode();
            }
        }
        if (n == 0 && m == 0) vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool c6_flash_bridge_try_start(void)
{
    // Configure trigger button as input pull-up and sample
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_C6_BRIDGE_BTN_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    // Pressed assumed as low
    if (gpio_get_level(CONFIG_C6_BRIDGE_BTN_GPIO) == 0) {
        ESP_LOGW(TAG_BR, "Entering C6 flash bridge (button held)");
        xTaskCreatePinnedToCore(bridge_task, "c6_bridge", 4096, NULL, 10, NULL, 0);
        // Block forever; app should not proceed
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
        return true;
    }
    return false;
}

#else // !CONFIG_C6_BRIDGE_ENABLE

bool c6_flash_bridge_try_start(void)
{
    return false;
}

#endif // CONFIG_C6_BRIDGE_ENABLE
