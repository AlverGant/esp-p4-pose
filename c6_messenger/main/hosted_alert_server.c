#include "sdkconfig.h"
#include "hosted_alert_server.h"

#if CONFIG_HOSTED_ALERT_SERVER_ENABLE

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "esp_http_server.h"
#include "esp_log.h"

static const char *TAG = "hosted_http";

static hosted_alert_callback_t s_callback = NULL;
static httpd_handle_t s_server = NULL;

static void trim_trailing_whitespace(char *buf)
{
    if (!buf) {
        return;
    }
    size_t len = strlen(buf);
    while (len > 0) {
        unsigned char c = (unsigned char)buf[len - 1];
        if (!isspace(c)) {
            break;
        }
        buf[len - 1] = '\0';
        --len;
    }
}

static esp_err_t send_json_response(httpd_req_t *req, const char *status_text)
{
    httpd_resp_set_type(req, "application/json");
    if (!status_text) {
        return httpd_resp_send(req, NULL, 0);
    }
    return httpd_resp_sendstr(req, status_text);
}

static esp_err_t alert_post_handler(httpd_req_t *req)
{
    if (!s_callback) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "callback not ready");
        return ESP_OK;
    }

    size_t total = req->content_len;
    if (total == 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
        return ESP_OK;
    }
    if (total >= CONFIG_HOSTED_ALERT_MAX_BODY_LEN) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body too large");
        return ESP_OK;
    }

    char *payload = calloc(1, total + 1);
    if (!payload) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_OK;
    }

    size_t received = 0;
    while (received < total) {
        int ret = httpd_req_recv(req, payload + received, total - received);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            free(payload);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv failed");
            return ESP_OK;
        }
        received += (size_t)ret;
    }

    payload[received] = '\0';
    trim_trailing_whitespace(payload);

    hosted_alert_result_t result = s_callback(payload, received);

    switch (result) {
        case HOSTED_ALERT_RESULT_OK:
            send_json_response(req, "{\"status\":\"sent\"}");
            break;
        case HOSTED_ALERT_RESULT_COOLDOWN:
            httpd_resp_set_status(req, "429 Too Many Requests");
            send_json_response(req, "{\"status\":\"cooldown\"}");
            break;
        case HOSTED_ALERT_RESULT_IGNORED:
            httpd_resp_set_status(req, "204 No Content");
            send_json_response(req, NULL);
            break;
        default:
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "failed");
            break;
    }

    free(payload);
    return ESP_OK;
}

static esp_err_t health_get_handler(httpd_req_t *req)
{
    return httpd_resp_sendstr(req, "ok");
}

esp_err_t hosted_alert_server_start(hosted_alert_callback_t cb)
{
    if (!cb) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_server) {
        return ESP_OK;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_HOSTED_ALERT_SERVER_PORT;
    config.max_uri_handlers = 4;
    config.lru_purge_enable = true;

    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(err));
        s_server = NULL;
        return err;
    }

    s_callback = cb;

    const httpd_uri_t alert_uri = {
        .uri = "/alert",
        .method = HTTP_POST,
        .handler = alert_post_handler,
        .user_ctx = NULL,
    };
    err = httpd_register_uri_handler(s_server, &alert_uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register /alert handler: %s", esp_err_to_name(err));
        hosted_alert_server_stop();
        return err;
    }

    const httpd_uri_t health_uri = {
        .uri = "/health",
        .method = HTTP_GET,
        .handler = health_get_handler,
        .user_ctx = NULL,
    };
    err = httpd_register_uri_handler(s_server, &health_uri);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register /health handler: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "Hosted alert server listening on port %d", config.server_port);
    return ESP_OK;
}

void hosted_alert_server_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
    s_callback = NULL;
}

#else

esp_err_t hosted_alert_server_start(hosted_alert_callback_t cb)
{
    (void)cb;
    return ESP_ERR_NOT_SUPPORTED;
}

void hosted_alert_server_stop(void)
{
}

#endif
