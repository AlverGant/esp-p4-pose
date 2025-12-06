/**
 * @file at_mqtt.h
 * @brief MQTT client via ESP-AT commands
 */

#pragma once

#include "at_client.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// MQTT connection state
typedef enum {
    AT_MQTT_STATE_DISCONNECTED = 0,
    AT_MQTT_STATE_INIT,
    AT_MQTT_STATE_CONNECTING,
    AT_MQTT_STATE_CONNECTED,
    AT_MQTT_STATE_DISCONNECTING,
} at_mqtt_state_t;

// MQTT QoS levels
typedef enum {
    AT_MQTT_QOS_0 = 0,  // At most once
    AT_MQTT_QOS_1 = 1,  // At least once
    AT_MQTT_QOS_2 = 2,  // Exactly once
} at_mqtt_qos_t;

// MQTT configuration
typedef struct {
    const char *client_id;      // Client ID
    const char *username;       // Username (NULL if not needed)
    const char *password;       // Password (NULL if not needed)
    const char *cert_key_id;    // Certificate key ID (NULL for no TLS)
    const char *ca_id;          // CA certificate ID (NULL for no TLS)
    uint16_t keepalive;         // Keepalive interval in seconds (default: 120)
    bool clean_session;         // Clean session flag (default: true)
    const char *lwt_topic;      // Last Will topic (NULL to disable)
    const char *lwt_message;    // Last Will message
    at_mqtt_qos_t lwt_qos;      // Last Will QoS
    bool lwt_retain;            // Last Will retain flag
} at_mqtt_config_t;

#define AT_MQTT_CONFIG_DEFAULT() { \
    .client_id = "esp32p4", \
    .username = NULL, \
    .password = NULL, \
    .cert_key_id = NULL, \
    .ca_id = NULL, \
    .keepalive = 120, \
    .clean_session = true, \
    .lwt_topic = NULL, \
    .lwt_message = NULL, \
    .lwt_qos = AT_MQTT_QOS_0, \
    .lwt_retain = false, \
}

// Callback for received messages
typedef void (*at_mqtt_message_cb_t)(const char *topic, const uint8_t *data, size_t len, void *user_data);

/**
 * @brief Configure MQTT user properties
 * @param config MQTT configuration
 * @return AT response code
 */
at_response_t at_mqtt_user_config(const at_mqtt_config_t *config);

/**
 * @brief Connect to MQTT broker
 * @param link_id Link ID (0-4, usually 0)
 * @param scheme Connection scheme (1=TCP, 2=TLS no cert, 3=TLS server cert, etc.)
 * @param host Broker hostname/IP
 * @param port Broker port (1883 for TCP, 8883 for TLS)
 * @param reconnect Enable auto-reconnect
 * @return AT response code
 */
at_response_t at_mqtt_connect(uint8_t link_id, uint8_t scheme, const char *host,
                               uint16_t port, bool reconnect);

/**
 * @brief Quick connect with simple parameters
 * @param host Broker hostname
 * @param port Port
 * @param client_id Client ID
 * @param username Username (NULL for none)
 * @param password Password (NULL for none)
 * @return AT response code
 */
at_response_t at_mqtt_connect_simple(const char *host, uint16_t port,
                                      const char *client_id,
                                      const char *username, const char *password);

/**
 * @brief Disconnect from MQTT broker
 * @param link_id Link ID
 * @return AT response code
 */
at_response_t at_mqtt_disconnect(uint8_t link_id);

/**
 * @brief Check if connected to MQTT broker
 * @return true if connected
 */
bool at_mqtt_is_connected(void);

/**
 * @brief Get MQTT connection state
 * @param link_id Link ID
 * @return MQTT state
 */
at_mqtt_state_t at_mqtt_get_state(uint8_t link_id);

/**
 * @brief Subscribe to topic
 * @param link_id Link ID
 * @param topic Topic string
 * @param qos QoS level
 * @return AT response code
 */
at_response_t at_mqtt_subscribe(uint8_t link_id, const char *topic, at_mqtt_qos_t qos);

/**
 * @brief Unsubscribe from topic
 * @param link_id Link ID
 * @param topic Topic string
 * @return AT response code
 */
at_response_t at_mqtt_unsubscribe(uint8_t link_id, const char *topic);

/**
 * @brief Publish message to topic
 * @param link_id Link ID
 * @param topic Topic string
 * @param data Message data
 * @param len Message length
 * @param qos QoS level
 * @param retain Retain flag
 * @return AT response code
 */
at_response_t at_mqtt_publish(uint8_t link_id, const char *topic,
                               const uint8_t *data, size_t len,
                               at_mqtt_qos_t qos, bool retain);

/**
 * @brief Publish string message to topic
 * @param topic Topic string
 * @param message Message string
 * @return AT response code
 */
at_response_t at_mqtt_publish_string(const char *topic, const char *message);

/**
 * @brief Publish JSON formatted message
 * @param topic Topic string
 * @param json_fmt JSON format string
 * @param ... Format arguments
 * @return AT response code
 */
at_response_t at_mqtt_publish_json(const char *topic, const char *json_fmt, ...);

/**
 * @brief Set message callback
 * @param callback Callback function
 * @param user_data User data for callback
 */
void at_mqtt_set_message_callback(at_mqtt_message_cb_t callback, void *user_data);

/**
 * @brief Publish raw data (binary)
 * @param link_id Link ID
 * @param topic Topic
 * @param data Binary data
 * @param len Data length
 * @param qos QoS
 * @param retain Retain
 * @return AT response code
 */
at_response_t at_mqtt_publish_raw(uint8_t link_id, const char *topic,
                                   const uint8_t *data, size_t len,
                                   at_mqtt_qos_t qos, bool retain);

#ifdef __cplusplus
}
#endif
