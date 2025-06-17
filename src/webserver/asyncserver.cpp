#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "LittleFS.h"
#include "sensors-africa-logo.h"

AsyncWebServer server(80);
void setup_webserver();

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
    server.begin();
}
