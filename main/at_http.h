/**
 * @file at_http.h
 * @brief HTTP client via ESP-AT commands
 *
 * Supports HTTP/HTTPS requests for APIs like Telegram
 */

#pragma once

#include "at_client.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// HTTP methods
typedef enum {
    AT_HTTP_METHOD_HEAD = 1,
    AT_HTTP_METHOD_GET = 2,
    AT_HTTP_METHOD_POST = 3,
    AT_HTTP_METHOD_PUT = 4,
    AT_HTTP_METHOD_DELETE = 5,
} at_http_method_t;

// HTTP content types
typedef enum {
    AT_HTTP_CONTENT_WWW_FORM = 0,
    AT_HTTP_CONTENT_JSON = 1,
    AT_HTTP_CONTENT_TEXT = 2,
    AT_HTTP_CONTENT_OCTET = 3,
    AT_HTTP_CONTENT_FORM_DATA = 4,
} at_http_content_type_t;

// HTTP transport type
typedef enum {
    AT_HTTP_TRANSPORT_TCP = 1,
    AT_HTTP_TRANSPORT_TLS = 2,
} at_http_transport_t;

// HTTP response
typedef struct {
    int status_code;
    size_t content_length;
    char *body;
    size_t body_len;
} at_http_response_t;

/**
 * @brief Perform HTTP request
 * @param method HTTP method
 * @param url Full URL (http:// or https://)
 * @param headers Optional headers string (NULL for default)
 * @param body Request body (NULL for no body)
 * @param body_len Body length
 * @param response Output response (must be freed by caller)
 * @param timeout_ms Timeout
 * @return AT response code
 */
at_response_t at_http_request(at_http_method_t method, const char *url,
                               const char *headers, const uint8_t *body, size_t body_len,
                               at_http_response_t *response, uint32_t timeout_ms);

/**
 * @brief Simple HTTP GET
 * @param url URL
 * @param response Response buffer
 * @param response_len Response buffer length
 * @return AT response code
 */
at_response_t at_http_get(const char *url, char *response, size_t response_len);

/**
 * @brief HTTP POST with JSON body
 * @param url URL
 * @param json_body JSON string
 * @param response Response buffer
 * @param response_len Response buffer length
 * @return AT response code
 */
at_response_t at_http_post_json(const char *url, const char *json_body,
                                 char *response, size_t response_len);

/**
 * @brief HTTP POST with form data
 * @param url URL
 * @param form_data Form data string (key1=value1&key2=value2)
 * @param response Response buffer
 * @param response_len Response buffer length
 * @return AT response code
 */
at_response_t at_http_post_form(const char *url, const char *form_data,
                                 char *response, size_t response_len);

/**
 * @brief Free HTTP response
 * @param response Response to free
 */
void at_http_response_free(at_http_response_t *response);

// ============ Telegram API Helpers ============

/**
 * @brief Send Telegram text message
 * @param bot_token Bot token
 * @param chat_id Chat ID
 * @param message Message text
 * @return AT response code
 */
at_response_t at_telegram_send_message(const char *bot_token, const char *chat_id,
                                        const char *message);

/**
 * @brief Send Telegram message with formatting
 * @param bot_token Bot token
 * @param chat_id Chat ID
 * @param parse_mode Parse mode ("HTML" or "Markdown")
 * @param message Formatted message
 * @return AT response code
 */
at_response_t at_telegram_send_message_fmt(const char *bot_token, const char *chat_id,
                                            const char *parse_mode, const char *message);

/**
 * @brief Send Telegram photo (JPEG)
 * @param bot_token Bot token
 * @param chat_id Chat ID
 * @param jpeg_data JPEG image data
 * @param jpeg_len JPEG data length
 * @param caption Photo caption (NULL for none)
 * @return AT response code
 */
at_response_t at_telegram_send_photo(const char *bot_token, const char *chat_id,
                                      const uint8_t *jpeg_data, size_t jpeg_len,
                                      const char *caption);

/**
 * @brief Get Telegram bot updates (for testing)
 * @param bot_token Bot token
 * @param response Response buffer
 * @param response_len Response buffer length
 * @return AT response code
 */
at_response_t at_telegram_get_updates(const char *bot_token,
                                       char *response, size_t response_len);

#ifdef __cplusplus
}
#endif
