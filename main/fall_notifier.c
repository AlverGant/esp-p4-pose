#include "fall_notifier.h"
#include "net_telegram.h"
#include "coproc_uart.h"
#include "coproc_sdio.h"
#include "pose_overlay.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <ctype.h>
#include <stdio.h>

static const char *TAG_NOTIF = "notify";

static SemaphoreHandle_t s_send_mutex;
static int64_t s_last_sent_us = 0;
static int64_t s_cooldown_us = 0;
static bool s_init_done = false;
static portMUX_TYPE s_init_spin = portMUX_INITIALIZER_UNLOCKED;

static void fall_notifier_lazy_init(void)
{
    if (s_init_done) {
        return;
    }

    portENTER_CRITICAL(&s_init_spin);
    if (!s_init_done) {
        s_send_mutex = xSemaphoreCreateMutex();
        if (!s_send_mutex) {
            ESP_LOGE(TAG_NOTIF, "Falha ao criar mutex do notifier");
        }

        int cooldown_s = 0;
#if CONFIG_TELEGRAM_ENABLE
        cooldown_s = CONFIG_TELEGRAM_COOLDOWN_SEC;
#elif CONFIG_COPROC_UART_ENABLE
        cooldown_s = CONFIG_COPROC_COOLDOWN_SEC;
#else
        cooldown_s = 0;
#endif
        s_cooldown_us = (int64_t)cooldown_s * 1000000LL;
        s_last_sent_us = 0;
        s_init_done = true;
    }
    portEXIT_CRITICAL(&s_init_spin);
}

static void build_reason_tag(const char *source, char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return;
    }
    out[0] = '\0';
    if (!source) {
        return;
    }
    size_t j = 0;
    for (const char *s = source; *s && j < out_len - 1; ++s) {
        unsigned char c = (unsigned char)(*s);
        if (isalnum(c)) {
            out[j++] = (char)toupper(c);
        } else if (c == ' ' || c == '-' || c == '_' || c == '.') {
            if (j > 0 && out[j - 1] != '_') {
                out[j++] = '_';
            }
        }
    }
    while (j > 0 && out[j - 1] == '_') {
        --j;
    }
    out[j] = '\0';
}

static esp_err_t fall_notifier_send_locked(const char *source_human,
                                           int persons,
                                           int age_ms,
                                           int seq,
                                           bool urgent)
{
    const char *label = (source_human && *source_human) ? source_human : "Queda detectada!";

    int64_t now = esp_timer_get_time();
    if (!urgent && s_cooldown_us > 0 && (now - s_last_sent_us) < s_cooldown_us) {
        ESP_LOGI(TAG_NOTIF, "Cooldown ativo (%lld ms restantes)",
                 (long long)((s_cooldown_us - (now - s_last_sent_us)) / 1000));
        return ESP_ERR_INVALID_STATE;
    }

    bool delivered = false;
    esp_err_t last_err = ESP_FAIL;

    char msg[192];
    snprintf(msg, sizeof msg, "%s pessoas=%d, inferencia_ago=%dms, seq=%d",
             label, persons, age_ms, seq);

#if CONFIG_TELEGRAM_ENABLE
    esp_err_t err = telegram_send_text(msg);
    if (err == ESP_OK) {
        delivered = true;
        last_err = ESP_OK;
        ESP_LOGI(TAG_NOTIF, "Telegram enviado (%s)", label);
    } else {
        last_err = err;
        ESP_LOGW(TAG_NOTIF, "Falha ao enviar Telegram (%s)", esp_err_to_name(err));
    }
#endif

#if CONFIG_COPROC_UART_ENABLE
    {
        char line[224];
        int written = snprintf(line, sizeof line, CONFIG_COPROC_MSG_TEMPLATE, persons, age_ms, seq);
        if (written < 0) {
            line[0] = '\0';
            written = 0;
        }

        if (source_human && *source_human && written < (int)(sizeof line - 1)) {
            char tag[48];
            build_reason_tag(source_human, tag, sizeof tag);
            if (tag[0] != '\0') {
                snprintf(line + written, sizeof line - written, " reason=%s", tag);
            }
        }

        esp_err_t err_uart = coproc_uart_send_line(line);
        if (err_uart == ESP_OK) {
            delivered = true;
            last_err = ESP_OK;
            ESP_LOGI(TAG_NOTIF, "UART enviado: %s", line);
        } else {
            ESP_LOGW(TAG_NOTIF, "UART falhou (%s), tentando SDIO", esp_err_to_name(err_uart));
            esp_err_t err_sdio = coproc_sdio_send_line(line);
            if (err_sdio == ESP_OK) {
                delivered = true;
                last_err = ESP_OK;
                ESP_LOGI(TAG_NOTIF, "SDIO enviado: %s", line);
            } else {
                last_err = err_sdio;
                ESP_LOGW(TAG_NOTIF, "Falha ao enviar por UART e SDIO (%s)", esp_err_to_name(err_sdio));
            }
        }
    }
#endif

    if (delivered) {
        s_last_sent_us = now;
    }

#if !CONFIG_TELEGRAM_ENABLE && !CONFIG_COPROC_UART_ENABLE
    (void)delivered;
    (void)last_err;
    ESP_LOGW(TAG_NOTIF, "Nenhum backend de notificação habilitado");
    return ESP_ERR_INVALID_STATE;
#else
    return delivered ? ESP_OK : last_err;
#endif
}

esp_err_t fall_notifier_send_event(const char *source_human,
                                   int persons,
                                   int age_ms,
                                   int seq,
                                   bool urgent)
{
    fall_notifier_lazy_init();
    if (!s_send_mutex) {
        return ESP_ERR_NO_MEM;
    }
    if (xSemaphoreTake(s_send_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG_NOTIF, "Timeout ao aguardar mutex do notifier");
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = fall_notifier_send_locked(source_human, persons, age_ms, seq, urgent);
    xSemaphoreGive(s_send_mutex);
    return err;
}

static void notifier_task(void *arg)
{
    (void)arg;
    bool last_fall = false;

    for (;;) {
        bool fall = pose_overlay_is_fall_detected();
        int persons = 0, age_ms = 0, seq = 0;
        pose_overlay_get_stats(&persons, &age_ms, &seq);

        if (fall && !last_fall) {
            (void)fall_notifier_send_event("Queda detectada!", persons, age_ms, seq, false);
        }
        last_fall = fall;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void start_fall_notifier_task(void)
{
    fall_notifier_lazy_init();
    // SDIO/UART co-processor is initialized early in app_main()
    xTaskCreatePinnedToCore(notifier_task, "fall_notify", 4096, NULL, 3, NULL, 1);
}
