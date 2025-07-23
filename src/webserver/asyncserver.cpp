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
  server.begin();
}
