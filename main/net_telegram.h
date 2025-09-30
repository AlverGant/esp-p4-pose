#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize Wi-Fi STA and wait for IP
esp_err_t net_init_wifi(void);

// Send a Telegram text message using HTTPS
esp_err_t telegram_send_text(const char *text);

#ifdef __cplusplus
}
#endif

