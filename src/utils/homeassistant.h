/**
 * @file homeassistant.h
 * @brief Home Assistant integration over MQTT (WiFi first).
 *
 * Publishes sensor readings (PM1, PM2.5, PM10, temperature and humidity)
 * to a Home Assistant instance using the standard MQTT Discovery protocol.
 *
 * The integration uses its own dedicated PubSubClient/WiFiClient so it does
 * not interfere with the sensors.AFRICA telemetry MQTT client.
 *
 * Flow:
 *   1. On boot (after WiFi connects) discovery config messages are published
 *      with the retained flag so Home Assistant keeps the entities.
 *   2. Every time sensor data is read, the value is published to the entity's
 *      state topic.
 *
 * Configuration lives in src/global_configs.h (HA_* macros).
 * See HOME_ASSISTANT.md for full setup instructions.
 */

#ifndef HOMEASSISTANT_H
#define HOMEASSISTANT_H

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include "../global_configs.h"

// ---------------------------------------------------------------------------
// Home Assistant MQTT configuration defaults (override in global_configs.h)
// ---------------------------------------------------------------------------
#ifndef HA_ENABLE
#define HA_ENABLE 0
#endif

#ifndef HA_MQTT_BROKER
#define HA_MQTT_BROKER "" // e.g. "192.168.1.50" or "homeassistant.local"
#endif

#ifndef HA_MQTT_PORT
#define HA_MQTT_PORT 1883
#endif

#ifndef HA_MQTT_USERNAME
#define HA_MQTT_USERNAME ""
#endif

#ifndef HA_MQTT_PASSWORD
#define HA_MQTT_PASSWORD ""
#endif

#ifndef HA_DISCOVERY_PREFIX
#define HA_DISCOVERY_PREFIX "homeassistant"
#endif

#ifndef HA_DEVICE_NAME
#define HA_DEVICE_NAME "Sensors Africa Node"
#endif

#ifndef HA_DEVICE_MANUFACTURER
#define HA_DEVICE_MANUFACTURER "Code for Africa"
#endif

#ifndef HA_DEVICE_MODEL
#define HA_DEVICE_MODEL "ESP32-S3 Sensor Node"
#endif

// ---------------------------------------------------------------------------
// Dedicated MQTT client for Home Assistant
// ---------------------------------------------------------------------------
extern char esp_chipid[];

static WiFiClient haWifiClient;
static PubSubClient haMqttClient(haWifiClient);

static bool ha_configured = false;
static bool ha_discovery_published = false;
static unsigned long ha_last_reconnect_attempt = 0;
static const unsigned long HA_RECONNECT_INTERVAL_MS = 30000; // throttle reconnect attempts

/// @brief Stable, lowercase node identifier used in topics and unique IDs
static String haNodeId()
{
    String id = String(SENSOR_PREFIX) + String(esp_chipid);
    id.toLowerCase();
    return id;
}

/// @brief Base topic for this node's Home Assistant sensors
static String haBaseTopic()
{
    return String(HA_DISCOVERY_PREFIX) + "/sensor/" + haNodeId();
}

static String haDiscoveryTopic(const char *sensor_key)
{
    return haBaseTopic() + "/" + sensor_key + "/config";
}

static String haStateTopic(const char *sensor_key)
{
    return haBaseTopic() + "/" + sensor_key + "/state";
}

/// @brief Publish a single Home Assistant MQTT discovery config (retained)
static bool haPublishDiscoveryFor(const char *sensor_key, const char *name,
                                  const char *device_class, const char *unit,
                                  const char *icon = nullptr)
{
    if (!haMqttClient.connected())
        return false;

    JsonDocument doc;
    doc["name"] = name;
    doc["unique_id"] = haNodeId() + "_" + sensor_key;
    doc["state_topic"] = haStateTopic(sensor_key);
    doc["state_class"] = "measurement";

    if (device_class != nullptr && device_class[0] != '\0')
        doc["device_class"] = device_class;
    if (unit != nullptr && unit[0] != '\0')
        doc["unit_of_measurement"] = unit;
    if (icon != nullptr && icon[0] != '\0')
        doc["icon"] = icon;

    JsonObject device = doc["device"].to<JsonObject>();
    JsonArray identifiers = device["identifiers"].to<JsonArray>();
    identifiers.add(haNodeId());

    String deviceName = String(HA_DEVICE_NAME) + " " + String(esp_chipid);
    device["name"] = deviceName;
    device["manufacturer"] = HA_DEVICE_MANUFACTURER;
    device["model"] = HA_DEVICE_MODEL;

    char payload[512];
    if (serializeJson(doc, payload, sizeof(payload)) == 0)
        return false;

    String topic = haDiscoveryTopic(sensor_key);
    if (haMqttClient.publish(topic.c_str(), payload, true))
    {
        Serial.printf("[HA] Discovery published: %s -> %s\n", topic.c_str(), payload);
        return true;
    }
    return false;
}

/// @brief Publish discovery configs for every supported entity
static void haPublishDiscovery()
{
    if (!haMqttClient.connected())
        return;

    haPublishDiscoveryFor("temperature", "Temperature", "temperature", "°C", "mdi:thermometer");
    haPublishDiscoveryFor("humidity", "Humidity", "humidity", "%", "mdi:water-percent");
    haPublishDiscoveryFor("pm1", "PM1", "pm1", "µg/m³", "mdi:blur");
    haPublishDiscoveryFor("pm25", "PM2.5", "pm25", "µg/m³", "mdi:blur");
    haPublishDiscoveryFor("pm10", "PM10", "pm10", "µg/m³", "mdi:blur");

    ha_discovery_published = true;
    Serial.println("[HA] Discovery configs published");
}

/// @brief Connect the dedicated HA MQTT client to the Home Assistant broker
static bool haConnect()
{
    if (haMqttClient.connected())
        return true;

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("[HA] WiFi not connected, cannot connect to MQTT broker");
        return false;
    }

    if (HA_MQTT_BROKER[0] == '\0')
    {
        ha_configured = false;
        return false;
    }

    haMqttClient.setBufferSize(1024);
    haMqttClient.setServer(HA_MQTT_BROKER, HA_MQTT_PORT);

    String clientId = haNodeId() + "-ha";
    bool connected = false;

    if (HA_MQTT_USERNAME[0] != '\0')
        connected = haMqttClient.connect(clientId.c_str(), HA_MQTT_USERNAME, HA_MQTT_PASSWORD);
    else
        connected = haMqttClient.connect(clientId.c_str());

    if (connected)
    {
        Serial.printf("[HA] Connected to MQTT broker %s:%d as %s\n",
                      HA_MQTT_BROKER, HA_MQTT_PORT, clientId.c_str());
        ha_discovery_published = false;
        return true;
    }

    Serial.printf("[HA] MQTT connect failed, rc=%d\n", haMqttClient.state());
    return false;
}

/// @brief Make sure the HA MQTT connection is alive and discovery was sent
static bool haEnsureConnected()
{
    if (!ha_configured)
        return false;

    if (haMqttClient.connected())
    {
        if (!ha_discovery_published)
            haPublishDiscovery();
        return true;
    }

    unsigned long now = millis();
    if (now - ha_last_reconnect_attempt < HA_RECONNECT_INTERVAL_MS)
        return false;

    ha_last_reconnect_attempt = now;
    if (haConnect())
    {
        haPublishDiscovery();
        return true;
    }
    return false;
}

/// @brief Initialise the Home Assistant integration.
/// @return true if HA is enabled in firmware config and a broker is set
static bool haBegin()
{
    ha_configured = (HA_ENABLE != 0) && (HA_MQTT_BROKER[0] != '\0');

    if (!ha_configured)
    {
        Serial.println("[HA] Home Assistant integration disabled (set HA_ENABLE=1 and HA_MQTT_BROKER in global_configs.h)");
        return false;
    }

    Serial.printf("[HA] Home Assistant integration enabled -> broker %s:%d\n",
                  HA_MQTT_BROKER, HA_MQTT_PORT);
    return true;
}

/// @brief Publish a single sensor value to its Home Assistant state topic
static bool haPublishState(const char *sensor_key, float value)
{
    if (!haEnsureConnected())
        return false;

    if (isnan(value))
        return false;

    String topic = haStateTopic(sensor_key);
    char payload[32];
    snprintf(payload, sizeof(payload), "%.2f", value);

    if (haMqttClient.publish(topic.c_str(), payload))
    {
        Serial.printf("[HA] %s -> %s\n", sensor_key, payload);
        return true;
    }

    Serial.printf("[HA] Failed to publish %s\n", sensor_key);
    return false;
}

/// @brief Publish temperature and humidity readings to Home Assistant
static void haPublishClimate(float temperature, float humidity)
{
    if (!ha_configured)
        return;
    haPublishState("temperature", temperature);
    haPublishState("humidity", humidity);
}

/// @brief Publish particulate matter readings to Home Assistant
static void haPublishParticulates(float pm1, float pm25, float pm10)
{
    if (!ha_configured)
        return;
    haPublishState("pm1", pm1);
    haPublishState("pm25", pm25);
    haPublishState("pm10", pm10);
}

/// @brief Service routine - call from loop() to keep the HA client healthy
static void haLoop()
{
    if (!ha_configured)
        return;

    if (haMqttClient.connected())
    {
        haMqttClient.loop();
        if (!ha_discovery_published)
            haPublishDiscovery();
        return;
    }

    haEnsureConnected();
}

#endif // HOMEASSISTANT_H
