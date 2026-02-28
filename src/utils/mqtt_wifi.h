#ifndef MQTT_WIFI_H
#define MQTT_WIFI_H

#include <PubSubClient.h>
#include <WiFi.h>

// MQTT WiFi Global Variables
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// WiFi MQTT Status Variables
bool wifi_mqtt_connected = false;
bool wifi_mqtt_initialized = false;

/// @brief Connect to MQTT broker via WiFi
/// @param broker MQTT broker hostname/IP
/// @param port MQTT broker port
/// @param client_id MQTT client ID
/// @param username MQTT username (optional)
/// @param password MQTT password (optional)
/// @return true if connected successfully, false otherwise
static bool wifiMQTTConnect(const char *broker, uint16_t port, const char *client_id,
                            const char *username = nullptr, const char *password = nullptr)
{
    // Ensure WiFi is connected
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("wifiMQTTConnect: WiFi not connected");
        return false;
    }

    if (!mqttClient.setBufferSize(2048))
    {
        Serial.println("wifiMQTTConnect: Failed to allocate MQTT buffer");
        return false;
    }


    // Set server
    mqttClient.setServer(broker, port);

    // Set timeout for connecting
    int attempt_count = 0;
    int max_attempts = 5;

    Serial.print("wifiMQTTConnect: Connecting to MQTT broker ");
    Serial.print(broker);
    Serial.print(":");
    Serial.println(port);

    while (!mqttClient.connected() && attempt_count < max_attempts)
    {
        // Attempt to connect
        if (username && password)
        {
            if (mqttClient.connect(client_id, username, password))
            {
                Serial.println("wifiMQTTConnect: Connected to MQTT broker");
                wifi_mqtt_connected = true;
                return true;
            }
        }
        else
        {
            if (mqttClient.connect(client_id))
            {
                Serial.println("wifiMQTTConnect: Connected to MQTT broker");
                wifi_mqtt_connected = true;
                return true;
            }
        }

        Serial.print("wifiMQTTConnect: Failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" Retrying in 2 seconds");
        delay(2000);
        attempt_count++;
    }

    Serial.println("wifiMQTTConnect: Failed to connect to MQTT broker");
    wifi_mqtt_connected = false;
    return false;
}

/// @brief Disconnect from MQTT broker
static void wifiMQTTDisconnect()
{
    if (mqttClient.connected())
    {
        Serial.println("wifiMQTTDisconnect: Disconnecting from MQTT broker");
        mqttClient.disconnect();
        delay(500);
    }
    wifi_mqtt_connected = false;
}

/// @brief Publish message to MQTT topic
/// @param topic MQTT topic
/// @param payload Message payload
/// @param retain Whether to retain the message
/// @return true if published successfully, false otherwise
static bool wifiMQTTPublish(const char *topic, const char *payload, bool retain = false)
{
    if (!mqttClient.connected())
    {
        Serial.println("wifiMQTTPublish: Not connected to MQTT broker");
        return false;
    }

    Serial.print("wifiMQTTPublish: Publishing to topic ");
    Serial.print(topic);
    Serial.print(" - Payload size: ");
    Serial.print(strlen(payload));
    Serial.println(" bytes");

    if (mqttClient.publish(topic, payload, retain))
    {
        Serial.println("wifiMQTTPublish: Message published successfully");
        return true;
    }
    else
    {
        Serial.println("wifiMQTTPublish: Failed to publish message");
        return false;
    }
}

/// @brief Check and maintain MQTT connection
static void wifiMQTTLoop()
{
    if (mqttClient.connected())
    {
        mqttClient.loop();
    }
}

/// @brief Initialize and send telemetry via WiFi MQTT
/// @param broker MQTT broker hostname/IP
/// @param port MQTT broker port
/// @param topic MQTT topic for telemetry
/// @param client_id MQTT client ID
/// @param username MQTT username (optional)
/// @param password MQTT password (optional)
/// @param payload JSON payload to send
/// @param disconnect_after If true, disconnect after sending
/// @return true if successful, false otherwise
static bool wifiMQTTSendTelemetry(const char *broker, uint16_t port, const char *topic,
                                  const char *client_id, const char *payload,
                                  const char *username = nullptr, const char *password = nullptr,
                                  bool disconnect_after = true)
{
    // Ensure WiFi is connected
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("wifiMQTTSendTelemetry: WiFi not connected");
        return false;
    }

    // Connect to MQTT broker if not already connected
    if (!mqttClient.connected())
    {
        if (!wifiMQTTConnect(broker, port, client_id, username, password))
        {
            Serial.println("wifiMQTTSendTelemetry: Failed to connect to MQTT broker");
            return false;
        }
    }

    // Allow some time for the connection to establish
    delay(500);

    // Publish telemetry
    if (!wifiMQTTPublish(topic, payload, false))
    {
        Serial.println("wifiMQTTSendTelemetry: Failed to publish telemetry");
        if (disconnect_after)
        {
            wifiMQTTDisconnect();
        }
        return false;
    }

    Serial.println("wifiMQTTSendTelemetry: Telemetry sent successfully");

    // Disconnect if requested
    if (disconnect_after)
    {
        delay(500);
        wifiMQTTDisconnect();
    }

    return true;
}

#endif // MQTT_WIFI_H
