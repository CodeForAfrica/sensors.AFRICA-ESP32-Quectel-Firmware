/**
 * @file homeassistant.h
 * @brief Home Assistant integration over MQTT (WiFi first).
 *
 * Publishes sensor readings (PM1, PM2.5, PM10, temperature and humidity)
 * to a Home Assistant instance using the standard MQTT Discovery protocol.
 *
 * The integration uses a dedicated PubSubClient/WiFiClient so it does not
 * interfere with the sensors.AFRICA telemetry MQTT client.
 *
 * Flow:
 *   1. On boot (after WiFi connects) discovery config messages are published
 *      with the retained flag so Home Assistant keeps the entities.
 *   2. Every time sensor data is read, the value is published to the entity's
 *      state topic.
 *
 * HomeAssistantManager owns the connection strings (broker, username,
 * password, port) and the connection status. It does not read the HA_* global
 * config macros - feed it with setConfig() using values from DeviceConfig.
 * See HOME_ASSISTANT.md for full setup instructions.
 */

#ifndef HOMEASSISTANT_H
#define HOMEASSISTANT_H

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Identity / topic defaults (these are NOT connection settings)
// ---------------------------------------------------------------------------
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

/**
 * @brief Home Assistant MQTT manager.
 *
 * Tracks the connection status and the connection strings (broker, username,
 * password and port) in its own state instead of relying on global configs.
 */
class HomeAssistantManager
{
public:
    enum class Status : uint8_t
    {
        Disabled = 0, // no broker configured / integration disabled
        Disconnected, // configured but not connected to the broker
        Connecting,   // a connection attempt is in progress
        Connected     // connected to the broker
    };

    HomeAssistantManager()
    {
        _nodeId[0] = '\0';
        _broker[0] = '\0';
        _username[0] = '\0';
        _password[0] = '\0';
        _port = 1883;
        _status = Status::Disabled;
        _discoveryPublished = false;
        _lastReconnectAttempt = 0;
        _mqtt.setClient(_wifi);
    }

    /// @brief Set the node identifier used in topics and unique IDs.
    /// The node id is built from the device prefix and chip id, both passed
    /// in by the caller instead of being read from global configs.
    void setNodeId(const char *prefix, const char *chipId)
    {
        String id = String(prefix ? prefix : "") + String(chipId ? chipId : "");
        id.toLowerCase();

        strncpy(_nodeId, id.c_str(), sizeof(_nodeId) - 1);
        _nodeId[sizeof(_nodeId) - 1] = '\0';
    }

    /// @brief Set the connection strings tracked by the manager.
    /// @param broker MQTT broker hostname/IP (empty disables the integration)
    /// @param port MQTT broker port
    /// @param username MQTT username (optional)
    /// @param password MQTT password (optional)
    void setConfig(const char *broker, uint16_t port = 1883,
                   const char *username = nullptr, const char *password = nullptr)
    {
        const char *b = broker ? broker : "";
        const char *u = username ? username : "";
        const char *p = password ? password : "";

        bool changed = false;
        changed |= strcmp(_broker, b) != 0;
        changed |= _port != port;
        changed |= strcmp(_username, u) != 0;
        changed |= strcmp(_password, p) != 0;

        strncpy(_broker, b, sizeof(_broker) - 1);
        _broker[sizeof(_broker) - 1] = '\0';
        _port = port;
        strncpy(_username, u, sizeof(_username) - 1);
        _username[sizeof(_username) - 1] = '\0';
        strncpy(_password, p, sizeof(_password) - 1);
        _password[sizeof(_password) - 1] = '\0';

        if (changed)
        {
            if (_mqtt.connected())
                _mqtt.disconnect();
            _discoveryPublished = false;
            _lastReconnectAttempt = 0;
            _status = (_broker[0] == '\0') ? Status::Disabled : Status::Disconnected;
        }
    }

    /// @brief (Re)initialise the manager from its stored connection strings
    bool begin()
    {
        _status = (_broker[0] == '\0') ? Status::Disabled : Status::Disconnected;

        if (_status == Status::Disabled)
        {
            Serial.println("[HA] Home Assistant integration disabled (no broker configured)");
            return false;
        }

        Serial.printf("[HA] Home Assistant enabled -> broker %s:%u\n", _broker, _port);
        return true;
    }

    /// @brief Connect to the broker and publish discovery configs on success
    bool connect()
    {
        if (_mqtt.connected())
        {
            _status = Status::Connected;
            if (!_discoveryPublished)
                publishDiscovery();
            return true;
        }

        if (WiFi.status() != WL_CONNECTED)
        {
            _status = Status::Disconnected;
            Serial.println("[HA] WiFi not connected, cannot connect to MQTT broker");
            return false;
        }

        if (_broker[0] == '\0')
        {
            _status = Status::Disabled;
            return false;
        }

        _status = Status::Connecting;
        _mqtt.setBufferSize(1024);
        _mqtt.setServer(_broker, _port);

        String clientId = nodeId() + "-ha";
        bool ok = false;
        if (_username[0] != '\0')
            ok = _mqtt.connect(clientId.c_str(), _username, _password);
        else
            ok = _mqtt.connect(clientId.c_str());

        if (ok)
        {
            _status = Status::Connected;
            _discoveryPublished = false;
            Serial.printf("[HA] Connected to MQTT broker %s:%u as %s\n",
                          _broker, _port, clientId.c_str());
            publishDiscovery();
        }
        else
        {
            _status = Status::Disconnected;
            Serial.printf("[HA] MQTT connect failed, rc=%d\n", _mqtt.state());
        }

        return ok;
    }

    /// @brief Service routine - call from loop() to keep the connection alive
    void loop()
    {
        if (_status == Status::Disabled)
            return;

        if (_mqtt.connected())
        {
            _status = Status::Connected;
            _mqtt.loop();
            if (!_discoveryPublished)
                publishDiscovery();
            return;
        }

        _status = Status::Disconnected;

        unsigned long now = millis();
        if (now - _lastReconnectAttempt >= RECONNECT_INTERVAL_MS)
        {
            _lastReconnectAttempt = now;
            connect();
        }
    }

    /// @brief Publish a single sensor value to its Home Assistant state topic
    bool publishState(const char *sensor_key, float value)
    {
        if (_status != Status::Connected || !_mqtt.connected())
            return false;

        if (isnan(value))
            return false;

        String topic = stateTopic(sensor_key);
        char payload[32];
        snprintf(payload, sizeof(payload), "%.2f", value);

        if (_mqtt.publish(topic.c_str(), payload))
        {
            Serial.printf("[HA] %s -> %s\n", sensor_key, payload);
            return true;
        }

        Serial.printf("[HA] Failed to publish %s\n", sensor_key);
        return false;
    }

    /// @brief Publish temperature and humidity readings
    void publishClimate(float temperature, float humidity)
    {
        if (_status != Status::Connected)
            return;
        publishState("temperature", temperature);
        publishState("humidity", humidity);
    }

    /// @brief Publish particulate matter readings
    void publishParticulates(float pm1, float pm25, float pm10)
    {
        if (_status != Status::Connected)
            return;
        publishState("pm1", pm1);
        publishState("pm25", pm25);
        publishState("pm10", pm10);
    }

    /// @brief Publish retained discovery configs for all entities
    void publishDiscovery()
    {
        if (!_mqtt.connected())
            return;

        publishDiscoveryFor("temperature", "Temperature", "temperature", "°C", "mdi:thermometer");
        publishDiscoveryFor("humidity", "Humidity", "humidity", "%", "mdi:water-percent");
        publishDiscoveryFor("pm1", "PM1", "pm1", "µg/m³", "mdi:blur");
        publishDiscoveryFor("pm25", "PM2.5", "pm25", "µg/m³", "mdi:blur");
        publishDiscoveryFor("pm10", "PM10", "pm10", "µg/m³", "mdi:blur");

        _discoveryPublished = true;
        Serial.println("[HA] Discovery configs published");
    }

    // Accessors
    Status status() const { return _status; }
    bool isConfigured() const { return _status != Status::Disabled; }
    bool isConnected() { return _status == Status::Connected && _mqtt.connected(); }
    const char *broker() const { return _broker; }
    uint16_t port() const { return _port; }
    const char *username() const { return _username; }
    const char *password() const { return _password; }

private:
    String nodeId() const
    {
        return String(_nodeId);
    }

    String baseTopic() const
    {
        return String(HA_DISCOVERY_PREFIX) + "/sensor/" + nodeId();
    }

    String discoveryTopic(const char *sensor_key) const
    {
        return baseTopic() + "/" + sensor_key + "/config";
    }

    String stateTopic(const char *sensor_key) const
    {
        return baseTopic() + "/" + sensor_key + "/state";
    }

    bool publishDiscoveryFor(const char *sensor_key, const char *name,
                             const char *device_class, const char *unit,
                             const char *icon = nullptr)
    {
        if (!_mqtt.connected())
            return false;

        JsonDocument doc;
        doc["name"] = name;
        doc["unique_id"] = nodeId() + "_" + sensor_key;
        doc["state_topic"] = stateTopic(sensor_key);
        doc["state_class"] = "measurement";

        if (device_class != nullptr && device_class[0] != '\0')
            doc["device_class"] = device_class;
        if (unit != nullptr && unit[0] != '\0')
            doc["unit_of_measurement"] = unit;
        if (icon != nullptr && icon[0] != '\0')
            doc["icon"] = icon;

        JsonObject device = doc["device"].to<JsonObject>();
        JsonArray identifiers = device["identifiers"].to<JsonArray>();
        identifiers.add(nodeId());

        String deviceName = String(HA_DEVICE_NAME) + " " + nodeId();
        device["name"] = deviceName;
        device["manufacturer"] = HA_DEVICE_MANUFACTURER;
        device["model"] = HA_DEVICE_MODEL;

        char payload[512];
        if (serializeJson(doc, payload, sizeof(payload)) == 0)
            return false;

        String topic = discoveryTopic(sensor_key);
        if (_mqtt.publish(topic.c_str(), payload, true))
        {
            Serial.printf("[HA] Discovery published: %s -> %s\n", topic.c_str(), payload);
            return true;
        }
        return false;
    }

    // Node id + connection strings and status - owned by the manager
    char _nodeId[48];
    char _broker[64];
    char _username[32];
    char _password[32];
    uint16_t _port;
    Status _status;

    WiFiClient _wifi;
    PubSubClient _mqtt;
    bool _discoveryPublished;
    unsigned long _lastReconnectAttempt;

    static const unsigned long RECONNECT_INTERVAL_MS = 30000;
};

static HomeAssistantManager haManager;

#endif // HOMEASSISTANT_H
