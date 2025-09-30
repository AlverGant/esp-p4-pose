#pragma once

#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HOSTED_ALERT_RESULT_OK = 0,
    HOSTED_ALERT_RESULT_COOLDOWN,
    HOSTED_ALERT_RESULT_IGNORED,
    HOSTED_ALERT_RESULT_ERROR,
} hosted_alert_result_t;

typedef hosted_alert_result_t (*hosted_alert_callback_t)(const char *message, size_t message_len);

esp_err_t hosted_alert_server_start(hosted_alert_callback_t cb);
void hosted_alert_server_stop(void);

#ifdef __cplusplus
}
#endif
