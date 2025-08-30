#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "LittleFS.h"
#include "sensors-africa-logo.h"
#include "asyncserver.h"
#include <ArduinoJson.h>
#include "../utils/deviceconfig.h"
#include "../utils/wifi.h"

AsyncWebServer server(80);
extern struct_wifiInfo *wifiInfo;
extern uint8_t count_wifiInfo;
extern JsonDocument getCurrentSensorData();

void setup_webserver()
{
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/config.html"); });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html"); });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/config.html"); });
  server.on("/sensors_logo.png", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "image/png", SENSORSAFRICA_LOGO, SENSORSAFRICA_LOGO_PNG_SIZE); });
  server.on("/icons/wifi.svg", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/icons/wifi.svg", "image/svg+xml"); });
  server.on("/icons/simcard.svg", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/icons/simcard.svg", "image/svg+xml"); });
  server.on("/icons/lock.svg", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/icons/lock.svg", "image/svg+xml"); });
  server.on("/icons/cell_tower.svg", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/icons/cell_tower.svg", "image/svg+xml"); });
  server.on("/device-info.json", [](AsyncWebServerRequest *request)
            {
        JsonDocument data=getDeviceConfig();
        String data_str;
        serializeJson(data,data_str);
        request->send(200,"application/json",data_str); });
  server.on("/available-hotspots", HTTP_GET, [](AsyncWebServerRequest *request)
            {
                  JsonDocument doc;
                  Serial.print("Wifi hotspots: ");
                  Serial.println(count_wifiInfo);
                  for (uint8_t i = 0; i < count_wifiInfo; i++)
                  {
                    Serial.println(wifiInfo[i].ssid);
                    JsonObject SSID = doc[wifiInfo[i].ssid].to<JsonObject>();
                    SSID["rssi"] = wifiInfo[i].RSSI;
                    SSID["encType"] = wifiInfo[i].encryptionType;
                  }

                  String hotspots;
                  serializeJsonPretty(doc,Serial);
                  serializeJson(doc, hotspots);
                  request->send(200,"application/json",hotspots); });

  server.on("/save-config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
            {
              JsonDocument new_config;
              DeserializationError error = deserializeJson(new_config, data, len);
              if (error)
              {
                request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
                return;
              }
              request->send(200, "application/json", "{\"status\":\"Config received\"}");

              JsonDocument current_config = getDeviceConfig();

              size_t numKeys = 0;
              for (JsonPair kv : new_config.as<JsonObject>())
              {
                numKeys++;
              }

              // String keyNames = "[";
              // bool first = true;
              // for (JsonPair kv : current_config.as<JsonObject>()) {
              //   if (!first) keyNames += ",";
              //   keyNames += "\"" + String(kv.key().c_str()) + "\"";
              //   first = false;
              // }
              // keyNames += "]";
              // Serial.println("Config keys: " + keyNames);

              // Create a String array to hold the key names
              String keyNamesArr[numKeys];
              size_t idx = 0;
              for (JsonPair kv : new_config.as<JsonObject>())
              {
                keyNamesArr[idx++] = String(kv.key().c_str());
              }
              // for (JsonPair kv : current_config.as<JsonObject>()) {
              //   keyArray.add(kv.key().c_str());
              // }
              Serial.println("Key\tCurrentValue\tNewValue");
              for (int i = 0; i < numKeys; i++)
              {
                Serial.print(keyNamesArr[i]);
                Serial.print("\t");
                Serial.print(String(current_config[keyNamesArr[i]]));
                Serial.print("\t");
                Serial.println(String(new_config[keyNamesArr[i]]));

                // check if value is empty
                if (new_config[keyNamesArr[i]].isNull() || String(new_config[keyNamesArr[i]]).length() == 0)
                {
                  // skip empty values
                  continue;
                }
                current_config[keyNamesArr[i]] = new_config[keyNamesArr[i]];
              }

              // Display updated config
              Serial.println("\nUpdated Config:");
              serializeJsonPretty(current_config, Serial);
              saveConfig(current_config); });

  server.on("/sensor-data", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              // get JSON sensor data
              JsonDocument sensor_data = getCurrentSensorData();
              String sensor_data_str;
              serializeJson(sensor_data, sensor_data_str);
              request->send(200, "application/json", sensor_data_str); });

  server.begin();
}
