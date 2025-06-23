#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "LittleFS.h"
#include "sensors-africa-logo.h"
#include "asyncserver.h"
#include <ArduinoJson.h>
#include "../utils/deviceconfig.h"
#include "../utils/wifi.h"

AsyncWebServer server(80);

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
    server.on("/device-config.json", [](AsyncWebServerRequest *request)
              {
        JsonDocument data=getDeviceConfig();
        String data_str;
        serializeJson(data,data_str);
        request->send(200,"application/json",data_str); });
    server.on("/available-hotspots", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                  JsonDocument doc;

                  for (uint8_t i = 0; i < count_wifiInfo; i++)
                  {   
                    JsonObject SSID = doc[wifiInfo[i].ssid].to<JsonObject>();
                    SSID["rssi"] = wifiInfo[i].RSSI;
                    SSID["encType"] = wifiInfo[i].encryptionType;
                  }

                  String hotspots;
                  serializeJson(doc, hotspots);
                  request->send(200,"application/json",hotspots); });
    server.begin();
}
