#include "net_telegram.h"
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "soc/soc_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#if SOC_WIFI_SUPPORTED
#include "esp_wifi.h"
#endif

static const char *TAG_NET = "net";

// Simple Wi-Fi connection helper (STA)
esp_err_t net_init_wifi(void)
{
#if SOC_WIFI_SUPPORTED
    ESP_LOGI(TAG_NET, "Init netif/event loop");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_cfg = { 0 };
    strncpy((char *)wifi_cfg.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_cfg.sta.ssid)-1);
    strncpy((char *)wifi_cfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_cfg.sta.password)-1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // reasonable default

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_NET, "Connecting to Wi-Fi SSID='%s'", CONFIG_WIFI_SSID);

    // Wait for IP
    esp_netif_ip_info_t ip;
    for (int i = 0; i < 100; ++i) { // ~10s total
        vTaskDelay(pdMS_TO_TICKS(100));
        if (esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip) == ESP_OK && ip.ip.addr != 0) {
            ESP_LOGI(TAG_NET, "Got IP");
            return ESP_OK;
        }
    }
    ESP_LOGW(TAG_NET, "Wi-Fi IP not acquired yet; continuing");
    return ESP_OK;
#else
    ESP_LOGW(TAG_NET, "Wi-Fi not supported on this SoC without companion radio");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

// Telegram HTTPS POST (JSON) to sendMessage
esp_err_t telegram_send_text(const char *text)
{
#if !CONFIG_TELEGRAM_ENABLE
    return ESP_ERR_INVALID_STATE;
#else
    // If SoC has no native Wi-Fi, this requires ESP-Hosted/AT setup
#if !SOC_WIFI_SUPPORTED
    return ESP_ERR_NOT_SUPPORTED;
#endif
    if (!text || !*text) return ESP_ERR_INVALID_ARG;
    if (strlen(CONFIG_TELEGRAM_BOT_TOKEN) == 0 || strlen(CONFIG_TELEGRAM_CHAT_ID) == 0) {
        ESP_LOGW(TAG_NET, "Telegram token/chat_id not configured");
        return ESP_ERR_INVALID_STATE;
    }

    char url[256];
    snprintf(url, sizeof(url), "https://api.telegram.org/bot%ssendMessage", 
             CONFIG_TELEGRAM_BOT_TOKEN);

    // Build small JSON body. Escape quotes minimally.
    char safe_text[256];
    size_t j = 0; const char *s = text;
    while (*s && j < sizeof(safe_text) - 1) {
        if (*s == '"' && j < sizeof(safe_text) - 2) { safe_text[j++] = '\\'; safe_text[j++] = '"'; }
        else if (*s == '\n' && j < sizeof(safe_text) - 2) { safe_text[j++] = '\\'; safe_text[j++] = 'n'; }
        else { safe_text[j++] = *s; }
        ++s;
    }
    safe_text[j] = 0;

    char body[384];
    snprintf(body, sizeof(body), "{\"chat_id\":\"%s\",\"text\":\"%s\"}",
             CONFIG_TELEGRAM_CHAT_ID, safe_text);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) return ESP_FAIL;
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, body, strlen(body));
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG_NET, "Telegram status=%d", status);
        if (status != 200) err = ESP_FAIL;
    } else {
        ESP_LOGW(TAG_NET, "HTTP perform failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
    return err;
#endif
}
