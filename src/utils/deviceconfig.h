
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
    const char *new_config_file = "/config.json.new";
    String json_stringified = "";
    serializeJson(doc, json_stringified);
    writeFile(LittleFS, new_config_file, json_stringified.c_str());
    updateFileContents(LittleFS, "/config.json", new_config_file);
}

#endif