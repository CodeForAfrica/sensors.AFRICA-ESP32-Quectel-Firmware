
#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include <ArduinoJson.h>
#include <LittleFS.h>
#include "helpers.h"
#include "SD_handler.h"
#include "../global_configs.h"

static JsonDocument getDeviceConfig();
static void updateDeviceConfig();
static void saveConfig(JsonDocument &doc);
static void loadSavedDeviceConfigs();

enum ConfigurationState
{
    CONFIG_BOOT_INIT,              // Reading existing config
    CONFIG_CAPTIVE_PORTAL_ACTIVE,  // Waiting for user input
    CONFIG_CAPTIVE_PORTAL_TIMEOUT, // Auto-continue
    CONFIG_WIFI,
    CONFIG_GSM,
    CONFIG_APPLIED, // Configs loaded & applied
    CONFIG_COMPLETE // Ready for runtime
};

struct DeviceConfigState
{
    ConfigurationState state;
    unsigned long captivePortalStartTime;
    unsigned long captivePortalTimeoutMs; // 5-10 mins
    bool configurationRequired;           // False if valid config exists
    bool captivePortalAccessed;           // Tracks if user accessed it
    bool wifiConnected = false;
};

extern struct DeviceConfigState DeviceConfigState;

struct DeviceConfig
{
    char wifi_sta_ssid[64];
    char wifi_sta_pwd[64];
    char gsm_apn[32];
    char gsm_apn_pwd[32];
    char sim_pin[5];
};

extern struct DeviceConfig DeviceConfig;

static JsonDocument getDeviceConfig()
{
    JsonDocument doc;
    String config_data = readFile(LittleFS, "/config.json");
    if (config_data != "" && validateJson(config_data.c_str()))
    {
        deserializeJson(doc, config_data);
        return doc;
    }

    return doc;
}

static void updateDeviceConfig()
{

    JsonDocument config = getDeviceConfig();

    config["gsm_capable"] = gsm_capable;
    config["wifi_available"] = use_wifi;

    saveConfig(config);
}

static void saveConfig(JsonDocument &doc)
{
    // Helper: recursively merge src into dst ( adds new keys, updates differing values )
    static std::function<void(const JsonObjectConst &, JsonObject)> mergeJsonObjects =
        [&](const JsonObjectConst &src, JsonObject dst)
    {
        for (JsonPairConst kv : src)
        {
            const char *key = kv.key().c_str();
            JsonVariantConst v = kv.value();

            if (v.is<JsonObject>())
            {
                if (!dst[key].is<JsonObject>())
                {
                    dst[key] = src[key];
                }
                else
                {
                    JsonObject child = dst[key].as<JsonObject>();
                    mergeJsonObjects(v.as<JsonObjectConst>(), child);
                }
            }
            else
            {
                dst[key] = v;
            }
        }
    };

    const char *new_config_file = "/config.json.new";
    listFiles(LittleFS);
    String existing = readFile(LittleFS, "/config.json");
    String incoming;

    Serial.println("\nExisting config: ");
    Serial.println(existing);
    Serial.println("\nIncoming doc");
    serializeJsonPretty(doc, incoming);
    Serial.println(incoming);

    // If there is no existing config or it's invalid, just write the passed doc
    if (existing == "" || !validateJson(existing.c_str())) // ToDo: reduce this with either validateJson or DeserializationError
    {
        Serial.println("Invalid or empty json");
        const char *c_string = incoming.c_str();
        writeFile(LittleFS, "/config.json", c_string);
        return;
    }

    JsonDocument existingDoc;
    DeserializationError err = deserializeJson(existingDoc, existing);
    if (err)
    {
        Serial.println("\nDeserialization Error on existing doc");
        // Fallback: write the passed doc if existing couldn't be parsed //! Use this validaters with caution, they have failed in some tests.
        const char *c_string = incoming.c_str();
        writeFile(LittleFS, "/config.json", c_string);
        return;
    }

    JsonObject existingObj = existingDoc.as<JsonObject>();
    JsonObject newObj = doc.as<JsonObject>();

    if (!newObj.isNull())
    {
        mergeJsonObjects(newObj, existingObj);
    }

    String mergedString = "";
    serializeJson(existingDoc, mergedString);
    Serial.println("\nNew merged doc");
    Serial.println(mergedString);
    writeFile(LittleFS, new_config_file, mergedString.c_str());
    updateFileContents(LittleFS, "/config.json", new_config_file);
}

static void loadSavedDeviceConfigs()
{

    JsonDocument config = getDeviceConfig();

    if (config["ssid"] = !"")
    {
        strcpy(DeviceConfig.wifi_sta_ssid, config["ssid"]);
        strcpy(DeviceConfig.wifi_sta_pwd, config["wifiPwd"]);
    }
    if (config["apn"] = !"")
    {
        strcpy(DeviceConfig.wifi_sta_ssid, config["apn"]);
        strcpy(DeviceConfig.wifi_sta_pwd, config["apnPwd"]);
    }
}

#endif