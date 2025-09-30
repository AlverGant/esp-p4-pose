#include "fall_notifier.h"
#include "net_telegram.h"
#include "coproc_uart.h"
#include "coproc_sdio.h"
#include "pose_overlay.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <stdio.h>

static const char *TAG_NOTIF = "notify";

static void notifier_task(void *arg)
{
    (void)arg;
    bool last_fall = false;
    int64_t last_sent_us = 0;
    // Cooldown: choose source depending on path
    int cooldown_s = 0;
#if CONFIG_TELEGRAM_ENABLE
    cooldown_s = CONFIG_TELEGRAM_COOLDOWN_SEC;
#elif CONFIG_COPROC_UART_ENABLE
    cooldown_s = CONFIG_COPROC_COOLDOWN_SEC;
#else
    cooldown_s = 0;
#endif
    const int64_t cooldown = (int64_t)cooldown_s * 1000000LL;

    for (;;) {
        bool fall = pose_overlay_is_fall_detected();
        int persons = 0, age_ms = 0, seq = 0;
        pose_overlay_get_stats(&persons, &age_ms, &seq);

        int64_t now = esp_timer_get_time();
        bool can_send = (now - last_sent_us) >= cooldown;

        if (fall && !last_fall && can_send) {
            char msg[160];
            snprintf(msg, sizeof msg, "Queda detectada! pessoas=%d, inferencia_ago=%dms, seq=%d",
                     persons, age_ms, seq);
#if CONFIG_TELEGRAM_ENABLE
            {
                esp_err_t err = telegram_send_text(msg);
                if (err == ESP_OK) {
                    last_sent_us = now;
                    ESP_LOGI(TAG_NOTIF, "Telegram enviado");
                } else {
                    ESP_LOGW(TAG_NOTIF, "Falha ao enviar Telegram (%s)", esp_err_to_name(err));
                }
            }
#endif
#if CONFIG_COPROC_UART_ENABLE
            {
                char line[192];
                // Use template from Kconfig
                snprintf(line, sizeof line, CONFIG_COPROC_MSG_TEMPLATE, persons, age_ms, seq);

                // Try SDIO first, fallback to UART if SDIO fails
                esp_err_t err = coproc_sdio_send_line(line);
                if (err == ESP_OK) {
                    last_sent_us = now;
                    ESP_LOGI(TAG_NOTIF, "SDIO enviado: %s", line);
                } else {
                    // Fallback to UART
                    ESP_LOGW(TAG_NOTIF, "SDIO falhou, usando UART");
                    err = coproc_uart_send_line(line);
                    if (err == ESP_OK) {
                        last_sent_us = now;
                        ESP_LOGI(TAG_NOTIF, "UART enviado: %s", line);
                    } else {
                        ESP_LOGW(TAG_NOTIF, "Falha ao enviar (%s)", esp_err_to_name(err));
                    }
                }
            }
#endif
        }
        last_fall = fall;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void start_fall_notifier_task(void)
{
    // SDIO/UART co-processor is initialized early in app_main()
    xTaskCreatePinnedToCore(notifier_task, "fall_notify", 4096, NULL, 3, NULL, 1);
}
