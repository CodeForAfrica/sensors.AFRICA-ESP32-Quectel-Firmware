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
extern String listFiles(fs::FS &fs, String path);
extern char ROOT_DIR[24];
extern char AP_SSID[64];
String pendingFileList = "{}";
bool fileListReady = false;
AsyncWebServerRequest *pendingRequest = nullptr;
extern JsonDocument gsm_info;

void setup_webserver()
{
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/config.html"); });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/index.html"); });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request)
            { if(request->hasParam("skip")){DeviceConfigState.captivePortalAccessed=true;} //? we don't care about the value of skip, so no need to parse it
               else{request->send(LittleFS, "/config.html"); } });
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
  server.on("/device-id", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/plain", AP_SSID); });
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
                  // serializeJsonPretty(doc,Serial); // Debugging
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
              
              saveConfig(new_config); });

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

  // server.on("/list-files", HTTP_GET, [](AsyncWebServerRequest *request)
  //           { request->send(200, "application/json", listFiles(SD)); });

  server.on("/list-files", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              if (fileListReady && pendingRequest == nullptr)
              {
                // Previous result is ready, send it
                request->send(200, "application/json", pendingFileList);
                return;
              }

              if (pendingRequest != nullptr)
              {
                // Another request is already being processed
                request->send(503, "text/plain", "Server busy, try again later");
                return;
              }

              // Store the request and start the task
              pendingRequest = request;
              fileListReady = false;

              xTaskCreatePinnedToCore(
                  [](void *param)
                  {
                    // Get the file list
                    pendingFileList = listFiles(SD, String(ROOT_DIR));
                    fileListReady = true;

                    // Send the response
                    if (pendingRequest != nullptr)
                    {
                      pendingRequest->send(200, "application/json", pendingFileList);
                      pendingRequest = nullptr;
                    }

                    vTaskDelete(NULL);
                  },
                  "SDListTask",
                  8192,
                  nullptr,
                  1,
                  nullptr,
                  1); });

  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    // Check for the "file" query parameter
    if (!request->hasParam("file")) {
      request->send(400, "text/plain", "Missing 'file' parameter");
      return;
    }

    // Retrieve and sanitize the file path
    String filePath = request->getParam("file")->value();
    if (!filePath.startsWith("/")) {
      filePath = "/" + filePath;
    }

    // Verify file exists
    if (!LittleFS.exists(filePath)) {
      request->send(404, "text/plain", "File not found");
      return;
    }

    // Derive a filename for the Content-Disposition header
    String filename = filePath.substring(filePath.lastIndexOf('/') + 1);

    // Create response and force download
    AsyncWebServerResponse *response =
      request->beginResponse(LittleFS, filePath, "application/octet-stream");
    response->addHeader(
      "Content-Disposition",
      "attachment; filename=\"" + filename + "\""
    );

    request->send(response); });

  server.on("/gsm_info", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              String res;
              serializeJson(gsm_info,res);
              request->send(200,"application/json", res); });

  // Captive portal redirect
  server.onNotFound([](AsyncWebServerRequest *request)
                    { if(!DeviceConfigState.captivePortalAccessed) request->redirect("/config"); else request->send(404, "text/plain", "Not found"); });

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
