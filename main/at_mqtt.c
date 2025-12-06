/**
 * @file at_mqtt.c
 * @brief MQTT client via ESP-AT commands implementation
 */

#include "at_mqtt.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

static const char *TAG = "at_mqtt";

// State tracking
static at_mqtt_state_t s_mqtt_state = AT_MQTT_STATE_DISCONNECTED;
static at_mqtt_message_cb_t s_message_callback = NULL;
static void *s_callback_user_data = NULL;

at_response_t at_mqtt_user_config(const at_mqtt_config_t *config)
{
    if (!config || !config->client_id) {
        return AT_ERROR;
    }

    ESP_LOGI(TAG, "Configuring MQTT user: client_id=%s", config->client_id);

    // AT+MQTTUSERCFG=<LinkID>,<scheme>,<"client_id">,<"username">,<"password">,<cert_key_ID>,<CA_ID>,<"path">
    // Scheme: 1=TCP, 2=TLS (no cert), 3=TLS (server cert), 4=TLS (client cert), 5=TLS (both), 6=WS, 7=WSS
    return at_send_cmd_fmt(NULL, 0, 5000,
        "+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"",
        config->client_id,
        config->username ? config->username : "",
        config->password ? config->password : "");
}

at_response_t at_mqtt_connect(uint8_t link_id, uint8_t scheme, const char *host,
                               uint16_t port, bool reconnect)
{
    if (!host) {
        return AT_ERROR;
    }

    s_mqtt_state = AT_MQTT_STATE_CONNECTING;
    ESP_LOGI(TAG, "Connecting to MQTT broker: %s:%d", host, port);

    // AT+MQTTCONN=<LinkID>,<"host">,<port>,<reconnect>
    at_response_t ret = at_send_cmd_fmt(NULL, 0, 30000,
        "+MQTTCONN=%d,\"%s\",%d,%d",
        link_id, host, port, reconnect ? 1 : 0);

    if (ret == AT_OK) {
        s_mqtt_state = AT_MQTT_STATE_CONNECTED;
        ESP_LOGI(TAG, "MQTT connected to %s:%d", host, port);
    } else {
        s_mqtt_state = AT_MQTT_STATE_DISCONNECTED;
        ESP_LOGE(TAG, "MQTT connection failed: %s", at_response_to_str(ret));
    }

    return ret;
}

at_response_t at_mqtt_connect_simple(const char *host, uint16_t port,
                                      const char *client_id,
                                      const char *username, const char *password)
{
    at_response_t ret;

    // Configure user first
    at_mqtt_config_t config = AT_MQTT_CONFIG_DEFAULT();
    config.client_id = client_id ? client_id : "esp32p4";
    config.username = username;
    config.password = password;

    ret = at_mqtt_user_config(&config);
    if (ret != AT_OK) {
        return ret;
    }

    // Then connect
    return at_mqtt_connect(0, 1, host, port, true);
}

at_response_t at_mqtt_disconnect(uint8_t link_id)
{
    s_mqtt_state = AT_MQTT_STATE_DISCONNECTING;
    ESP_LOGI(TAG, "Disconnecting from MQTT broker");

    at_response_t ret = at_send_cmd_fmt(NULL, 0, 5000, "+MQTTCLEAN=%d", link_id);

    s_mqtt_state = AT_MQTT_STATE_DISCONNECTED;
    return ret;
}

bool at_mqtt_is_connected(void)
{
    char response[128];
    at_response_t ret = at_send_cmd("+MQTTCONN?", response, sizeof(response), 2000);

    if (ret == AT_OK) {
        // Parse +MQTTCONN:0,state,...
        // state: 0=not init, 1=set, 2=disconnected, 3=established, 4=connected, 5=disconnecting
        char *p = strstr(response, "+MQTTCONN:");
        if (p) {
            int link_id, state;
            if (sscanf(p, "+MQTTCONN:%d,%d", &link_id, &state) == 2) {
                if (state >= 3 && state <= 4) {
                    s_mqtt_state = AT_MQTT_STATE_CONNECTED;
                    return true;
                }
            }
        }
    }

    s_mqtt_state = AT_MQTT_STATE_DISCONNECTED;
    return false;
}

at_mqtt_state_t at_mqtt_get_state(uint8_t link_id)
{
    (void)link_id;
    at_mqtt_is_connected();  // Refresh state
    return s_mqtt_state;
}

at_response_t at_mqtt_subscribe(uint8_t link_id, const char *topic, at_mqtt_qos_t qos)
{
    if (!topic) {
        return AT_ERROR;
    }

    ESP_LOGI(TAG, "Subscribing to topic: %s (qos=%d)", topic, qos);

    return at_send_cmd_fmt(NULL, 0, 10000, "+MQTTSUB=%d,\"%s\",%d", link_id, topic, qos);
}

at_response_t at_mqtt_unsubscribe(uint8_t link_id, const char *topic)
{
    if (!topic) {
        return AT_ERROR;
    }

    ESP_LOGI(TAG, "Unsubscribing from topic: %s", topic);

    return at_send_cmd_fmt(NULL, 0, 5000, "+MQTTUNSUB=%d,\"%s\"", link_id, topic);
}

at_response_t at_mqtt_publish(uint8_t link_id, const char *topic,
                               const uint8_t *data, size_t len,
                               at_mqtt_qos_t qos, bool retain)
{
    if (!topic || !data || len == 0) {
        return AT_ERROR;
    }

    // For small messages, use MQTTPUB
    // For larger messages, use MQTTPUBRAW

    if (len <= 256) {
        // Use inline publish - need to escape the data
        // AT+MQTTPUB=<LinkID>,<"topic">,<"data">,<qos>,<retain>
        char cmd[512];
        int pos = snprintf(cmd, sizeof(cmd), "+MQTTPUB=%d,\"%s\",\"", link_id, topic);

        // Copy data, escaping special characters
        for (size_t i = 0; i < len && pos < (int)sizeof(cmd) - 20; i++) {
            char c = (char)data[i];
            if (c == '"' || c == '\\') {
                cmd[pos++] = '\\';
            }
            cmd[pos++] = c;
        }

        pos += snprintf(cmd + pos, sizeof(cmd) - pos, "\",%d,%d", qos, retain ? 1 : 0);

        ESP_LOGI(TAG, "Publishing to %s (%d bytes)", topic, (int)len);
        return at_send_cmd(cmd, NULL, 0, 10000);
    } else {
        // Use raw publish for larger data
        return at_mqtt_publish_raw(link_id, topic, data, len, qos, retain);
    }
}

at_response_t at_mqtt_publish_string(const char *topic, const char *message)
{
    if (!topic || !message) {
        return AT_ERROR;
    }

    return at_mqtt_publish(0, topic, (const uint8_t *)message, strlen(message),
                           AT_MQTT_QOS_1, false);
}

at_response_t at_mqtt_publish_json(const char *topic, const char *json_fmt, ...)
{
    char json[512];
    va_list args;
    va_start(args, json_fmt);
    vsnprintf(json, sizeof(json), json_fmt, args);
    va_end(args);

    return at_mqtt_publish_string(topic, json);
}

void at_mqtt_set_message_callback(at_mqtt_message_cb_t callback, void *user_data)
{
    s_message_callback = callback;
    s_callback_user_data = user_data;
}

at_response_t at_mqtt_publish_raw(uint8_t link_id, const char *topic,
                                   const uint8_t *data, size_t len,
                                   at_mqtt_qos_t qos, bool retain)
{
    if (!topic || !data || len == 0) {
        return AT_ERROR;
    }

    ESP_LOGI(TAG, "Publishing raw to %s (%d bytes)", topic, (int)len);

    // AT+MQTTPUBRAW=<LinkID>,<"topic">,<length>,<qos>,<retain>
    // Then send raw data after receiving ">"
    at_response_t ret = at_send_cmd_fmt(NULL, 0, 5000,
        "+MQTTPUBRAW=%d,\"%s\",%d,%d,%d",
        link_id, topic, (int)len, qos, retain ? 1 : 0);

    if (ret == AT_OK || ret == AT_UNKNOWN) {
        // Wait for prompt and send data
        ret = at_send_data(data, len, 10000);
    }

    return ret;
}
