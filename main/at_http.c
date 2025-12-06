/**
 * @file at_http.c
 * @brief HTTP client via ESP-AT commands implementation
 */

#include "at_http.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "at_http";

at_response_t at_http_request(at_http_method_t method, const char *url,
                               const char *headers, const uint8_t *body, size_t body_len,
                               at_http_response_t *response, uint32_t timeout_ms)
{
    if (!url) {
        return AT_ERROR;
    }

    if (response) {
        memset(response, 0, sizeof(*response));
    }

    // Determine transport type from URL
    at_http_transport_t transport = AT_HTTP_TRANSPORT_TCP;
    if (strncmp(url, "https://", 8) == 0) {
        transport = AT_HTTP_TRANSPORT_TLS;
    }

    // Content type based on headers or default
    at_http_content_type_t content_type = AT_HTTP_CONTENT_JSON;
    if (headers && strstr(headers, "application/x-www-form-urlencoded")) {
        content_type = AT_HTTP_CONTENT_WWW_FORM;
    } else if (headers && strstr(headers, "multipart/form-data")) {
        content_type = AT_HTTP_CONTENT_FORM_DATA;
    }

    ESP_LOGI(TAG, "HTTP %s %s", method == AT_HTTP_METHOD_GET ? "GET" : "POST", url);

    // AT+HTTPCLIENT=<opt>,<content-type>,<"url">,<"host">,<"path">,<transport>[,<"data">][,<"headers">]
    // opt: 1=HEAD, 2=GET, 3=POST, 4=PUT, 5=DELETE
    // This is complex - ESP-AT expects parsed URL components

    // Allocate buffers on heap to avoid stack overflow
    char *response_buf = malloc(1024);
    char *cmd = NULL;
    if (!response_buf) {
        ESP_LOGE(TAG, "Failed to allocate HTTP response buffer");
        return AT_ERROR;
    }

    if (body && body_len > 0) {
        // POST with data - need larger buffer
        cmd = malloc(2048);
        if (!cmd) {
            free(response_buf);
            ESP_LOGE(TAG, "Failed to allocate HTTP command buffer");
            return AT_ERROR;
        }
        const size_t cmd_size = 2048;
        int len = snprintf(cmd, cmd_size,
            "+HTTPCLIENT=%d,%d,\"%s\",\"\",\"\",%d,\"",
            method, content_type, url, transport);

        // Append body (escape quotes)
        for (size_t i = 0; i < body_len && len < (int)cmd_size - 50; i++) {
            char c = (char)body[i];
            if (c == '"' || c == '\\') {
                cmd[len++] = '\\';
            }
            if (c == '\n') {
                cmd[len++] = '\\';
                cmd[len++] = 'n';
            } else if (c == '\r') {
                // skip
            } else {
                cmd[len++] = c;
            }
        }

        if (headers) {
            len += snprintf(cmd + len, cmd_size - len, "\",\"%s\"", headers);
        } else {
            len += snprintf(cmd + len, cmd_size - len, "\"");
        }

        at_response_t ret = at_send_cmd(cmd, response_buf, 1024, timeout_ms);

        if (response) {
            // Parse response
            char *p = strstr(response_buf, "+HTTPCLIENT:");
            if (p) {
                // +HTTPCLIENT:<size>,<data>
                int size;
                if (sscanf(p, "+HTTPCLIENT:%d,", &size) == 1) {
                    response->content_length = size;
                    char *data = strchr(p, ',');
                    if (data) {
                        response->body = strdup(data + 1);
                        response->body_len = strlen(response->body);
                    }
                }
            }
        }

        free(cmd);
        free(response_buf);
        return ret;
    } else {
        // GET request
        at_response_t ret = at_send_cmd_fmt(response_buf, 1024, timeout_ms,
            "+HTTPCLIENT=%d,%d,\"%s\",\"\",\"\",%d",
            method, content_type, url, transport);

        if (response) {
            char *p = strstr(response_buf, "+HTTPCLIENT:");
            if (p) {
                int size;
                if (sscanf(p, "+HTTPCLIENT:%d,", &size) == 1) {
                    response->content_length = size;
                    char *data = strchr(p, ',');
                    if (data) {
                        response->body = strdup(data + 1);
                        response->body_len = strlen(response->body);
                    }
                }
            }
        }

        free(response_buf);
        return ret;
    }
}

at_response_t at_http_get(const char *url, char *response, size_t response_len)
{
    if (!url) {
        return AT_ERROR;
    }

    at_http_transport_t transport = AT_HTTP_TRANSPORT_TCP;
    if (strncmp(url, "https://", 8) == 0) {
        transport = AT_HTTP_TRANSPORT_TLS;
    }

    ESP_LOGI(TAG, "HTTP GET %s", url);

    return at_send_cmd_fmt(response, response_len, 30000,
        "+HTTPCLIENT=2,0,\"%s\",\"\",\"\",%d", url, transport);
}

at_response_t at_http_post_json(const char *url, const char *json_body,
                                 char *response, size_t response_len)
{
    if (!url || !json_body) {
        return AT_ERROR;
    }

    ESP_LOGI(TAG, "HTTP POST JSON %s", url);

    size_t json_len = strlen(json_body);

    // Build AT+HTTPCPOST command with Content-Type header
    // Format: AT+HTTPCPOST="url",length,header_count,"header1",...
    const size_t cmd_size = 512;
    char *cmd = malloc(cmd_size);
    if (!cmd) {
        ESP_LOGE(TAG, "Failed to allocate HTTP command buffer");
        return AT_ERROR;
    }

    snprintf(cmd, cmd_size,
        "+HTTPCPOST=\"%s\",%d,2,"
        "\"Content-Type: application/json\","
        "\"Connection: close\"",
        url, (int)json_len);

    ESP_LOGI(TAG, "Sending: AT%s", cmd);

    // Use data transfer function (waits for '>' then sends body)
    at_response_t result = at_send_cmd_with_data(
        cmd,
        (const uint8_t *)json_body, json_len,
        response, response_len,
        10000,   // 10 seconds for '>' prompt
        30000    // 30 seconds for HTTP response
    );

    free(cmd);

    // SEND OK is also a success response for data transfers
    if (result == AT_OK || result == AT_SEND_OK) {
        return AT_OK;
    }
    return result;
}

at_response_t at_http_post_form(const char *url, const char *form_data,
                                 char *response, size_t response_len)
{
    if (!url || !form_data) {
        return AT_ERROR;
    }

    at_http_transport_t transport = AT_HTTP_TRANSPORT_TCP;
    if (strncmp(url, "https://", 8) == 0) {
        transport = AT_HTTP_TRANSPORT_TLS;
    }

    ESP_LOGI(TAG, "HTTP POST form %s", url);

    return at_send_cmd_fmt(response, response_len, 30000,
        "+HTTPCLIENT=3,0,\"%s\",\"\",\"\",%d,\"%s\"", url, transport, form_data);
}

void at_http_response_free(at_http_response_t *response)
{
    if (response && response->body) {
        free(response->body);
        response->body = NULL;
    }
}

// ============ Telegram API ============

#define TELEGRAM_API_HOST "api.telegram.org"

at_response_t at_telegram_send_message(const char *bot_token, const char *chat_id,
                                        const char *message)
{
    if (!bot_token || !chat_id || !message) {
        return AT_ERROR;
    }

    ESP_LOGI(TAG, "Telegram: sending message to %s", chat_id);

    // Build URL
    char url[256];
    snprintf(url, sizeof(url), "https://%s/bot%s/sendMessage",
             TELEGRAM_API_HOST, bot_token);

    // Build JSON body
    char json[512];
    snprintf(json, sizeof(json),
        "{\"chat_id\":\"%s\",\"text\":\"%s\"}",
        chat_id, message);

    char response[256];
    at_response_t ret = at_http_post_json(url, json, response, sizeof(response));

    if (ret == AT_OK) {
        ESP_LOGI(TAG, "Telegram message sent successfully");
    } else {
        ESP_LOGE(TAG, "Telegram message failed: %s", at_response_to_str(ret));
    }

    return ret;
}

at_response_t at_telegram_send_message_fmt(const char *bot_token, const char *chat_id,
                                            const char *parse_mode, const char *message)
{
    if (!bot_token || !chat_id || !message) {
        return AT_ERROR;
    }

    ESP_LOGI(TAG, "Telegram: sending formatted message to %s", chat_id);

    char url[256];
    snprintf(url, sizeof(url), "https://%s/bot%s/sendMessage",
             TELEGRAM_API_HOST, bot_token);

    char json[1024];
    snprintf(json, sizeof(json),
        "{\"chat_id\":\"%s\",\"text\":\"%s\",\"parse_mode\":\"%s\"}",
        chat_id, message, parse_mode ? parse_mode : "HTML");

    char response[256];
    return at_http_post_json(url, json, response, sizeof(response));
}

at_response_t at_telegram_send_photo(const char *bot_token, const char *chat_id,
                                      const uint8_t *jpeg_data, size_t jpeg_len,
                                      const char *caption)
{
    if (!bot_token || !chat_id || !jpeg_data || jpeg_len == 0) {
        return AT_ERROR;
    }

    ESP_LOGI(TAG, "Telegram: sending photo to %s (%d bytes)", chat_id, (int)jpeg_len);

    // Use AT+HTTPCPOST with multipart/form-data
    // Structure:
    // --boundary\r\n
    // Content-Disposition: form-data; name="chat_id"\r\n\r\n
    // <chat_id>\r\n
    // --boundary\r\n
    // Content-Disposition: form-data; name="photo"; filename="photo.jpg"\r\n
    // Content-Type: image/jpeg\r\n\r\n
    // <binary jpeg data>
    // \r\n--boundary--\r\n

    #define BOUNDARY "----ESP32P4Boundary"

    // Build pre-data (before JPEG)
    char *pre_data = malloc(512);
    char *post_data = malloc(64);
    char *cmd = malloc(512);

    if (!pre_data || !post_data || !cmd) {
        ESP_LOGE(TAG, "Failed to allocate multipart buffers");
        free(pre_data);
        free(post_data);
        free(cmd);
        return AT_ERROR;
    }

    // Pre-data: chat_id field + photo field headers
    int pre_len = snprintf(pre_data, 512,
        "--" BOUNDARY "\r\n"
        "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n"
        "%s\r\n"
        "--" BOUNDARY "\r\n"
        "Content-Disposition: form-data; name=\"photo\"; filename=\"photo.jpg\"\r\n"
        "Content-Type: image/jpeg\r\n\r\n",
        chat_id);

    // Add caption if provided
    char caption_part[256] = "";
    int caption_len = 0;
    if (caption && strlen(caption) > 0) {
        caption_len = snprintf(caption_part, sizeof(caption_part),
            "\r\n--" BOUNDARY "\r\n"
            "Content-Disposition: form-data; name=\"caption\"\r\n\r\n"
            "%s",
            caption);
    }

    // Post-data: closing boundary
    int post_len = snprintf(post_data, 64, "\r\n--" BOUNDARY "--\r\n");

    // Total content length
    size_t total_len = pre_len + jpeg_len + caption_len + post_len;

    ESP_LOGI(TAG, "Multipart: pre=%d, jpeg=%d, caption=%d, post=%d, total=%d",
             pre_len, (int)jpeg_len, caption_len, post_len, (int)total_len);

    // Build URL
    char url[256];
    snprintf(url, sizeof(url), "https://%s/bot%s/sendPhoto",
             TELEGRAM_API_HOST, bot_token);

    // Build AT+HTTPCPOST command
    // AT+HTTPCPOST="url",length,header_count,"header1","header2"...
    snprintf(cmd, 512,
        "+HTTPCPOST=\"%s\",%d,2,"
        "\"Content-Type: multipart/form-data; boundary=" BOUNDARY "\","
        "\"Connection: close\"",
        url, (int)total_len);

    ESP_LOGI(TAG, "Sending: AT%s", cmd);

    // Build the complete multipart body
    uint8_t *body = malloc(total_len);
    if (!body) {
        ESP_LOGE(TAG, "Failed to allocate body buffer");
        free(pre_data);
        free(post_data);
        free(cmd);
        return AT_ERROR;
    }

    // Assemble the multipart body
    size_t pos = 0;
    memcpy(body + pos, pre_data, pre_len);
    pos += pre_len;
    memcpy(body + pos, jpeg_data, jpeg_len);
    pos += jpeg_len;
    if (caption_len > 0) {
        memcpy(body + pos, caption_part, caption_len);
        pos += caption_len;
    }
    memcpy(body + pos, post_data, post_len);
    pos += post_len;

    ESP_LOGI(TAG, "Assembled multipart body: %d bytes", (int)pos);

    // Use the AT client's proper data transfer function
    char response[256] = {0};
    at_response_t result = at_send_cmd_with_data(
        cmd,
        body, pos,
        response, sizeof(response),
        10000,   // 10 seconds for '>' prompt (TLS handshake can be slow)
        120000   // 120 seconds for HTTP response (large upload over TLS)
    );

    free(body);
    free(pre_data);
    free(post_data);
    free(cmd);

    // SEND OK is also a success response for data transfers
    if (result == AT_OK || result == AT_SEND_OK) {
        ESP_LOGI(TAG, "Photo upload completed successfully");
        return AT_OK;
    } else {
        ESP_LOGW(TAG, "Photo upload failed: %s", at_response_to_str(result));
        return result;
    }

    #undef BOUNDARY
}

at_response_t at_telegram_get_updates(const char *bot_token,
                                       char *response, size_t response_len)
{
    if (!bot_token) {
        return AT_ERROR;
    }

    char url[256];
    snprintf(url, sizeof(url), "https://%s/bot%s/getUpdates",
             TELEGRAM_API_HOST, bot_token);

    return at_http_get(url, response, response_len);
}
