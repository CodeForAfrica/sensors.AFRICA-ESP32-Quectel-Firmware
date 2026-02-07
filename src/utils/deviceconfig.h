
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

#endif