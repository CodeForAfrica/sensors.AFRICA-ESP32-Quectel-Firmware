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
extern String listFiles(fs::FS &fs);

void setup_webserver()
{
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/config.html"); });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html"); });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/config.html"); });
  server.on("/device-details.html", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/device-details.html"); });
  server.on("/ota.html", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/ota.html"); });
  server.on("/file-system.html", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/file-system.html"); });
  server.on("/advanced-settings.html", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/advanced-settings.html"); });
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

  server.on("/upload-firmware", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              request->send(200, "application/json", "{\"status\":\"Upload started\"}");
              Serial.println("Checking file system if firmware was uploaded");
              listFiles(LittleFS); }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
            {
              static File uploadFile;
              if (index == 0)
              {
                // Start upload
                Serial.printf("Upload started: %s\n", filename.c_str());
                uploadFile = LittleFS.open("/" + filename, "w");
                if (!uploadFile)
                {
                  Serial.println("Failed to open file for writing");
                  request->send(500, "application/json", "{\"error\":\"Failed to open file\"}");
                  return;
                }
              }
              if (uploadFile)
              {
                 uploadFile.write(data, len);
              }
              if (final)
              {
                if (uploadFile)
                {
                  uploadFile.close();
                  Serial.printf("Upload finished: %s\n", filename.c_str());
                  request->send(200, "application/json", "{\"status\":\"Upload successful\"}");
                }
                else
                {
                  request->send(500, "application/json", "{\"error\":\"File not open\"}");
                }
              } });

  server.on("/list-files", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "application/json", listFiles(SD)); });

  //! For comparison
  // void uploadFiles()
  // {
  //   // upload a new file to the SPIFFS
  //   HTTPUpload &upload = server.upload();
  //   if (upload.status == UPLOAD_FILE_START)
  //   {

  //     fname = upload.filename;
  //     if (!fname.startsWith("/"))
  //       fname = "/" + fname;
  //     Serial.print("Upload File Name: ");
  //     Serial.println(fname);
  //     uploadFile = SPIFFS.open(fname, "w"); // Open the file for writing in SPIFFS (create if it doesn't exist)
  //     if (uploadFile)
  //     {
  //       Serial.println("File opened");
  //     }
  //     // fname = String();
  //     Serial.print("fname: ");
  //     Serial.println(fname);
  //   }
  //   else if (upload.status == UPLOAD_FILE_WRITE)
  //   {
  //     if (uploadFile)
  //     {
  //       uploadFile.write(upload.buf, upload.currentSize);
  //       // Serial.println("written");
  //     }
  //   }

  //   else if (upload.status == UPLOAD_FILE_END)
  //   {
  //     if (uploadFile)
  //     {                     // If the file was successfully created
  //       uploadFile.close(); // Close the file again
  //       Serial.print("File Upload Size: ");
  //       Serial.println(upload.totalSize);
  //       String msg = "201: Successfully uploaded file ";
  //       msg += fname;
  //       server.send(200, "text/plain", msg);
  //       Serial.println(msg);

  //       if (fname == new_firmware_filename)
  //       {
  //         firmware_bin_saved = true;
  //       }
  //     }
  //     else
  //     {
  //       String err_msg = "500: failed creating file ";
  //       err_msg += fname;
  //       server.send(500, "text/plain", err_msg);
  //       Serial.println(err_msg);
  //     }
  //   }
  // }

  server.begin();
}
