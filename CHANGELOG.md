# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v1.4.0](https://github.com/CodeForAfrica/sensors.AFRICA-ESP32-Quectel-Firmware/releases/tag/v1.4.0) 2026-07-22

### Added
- `IS_LIVE` compile flag (`src/global_configs.h`) — toggles production/staging API URL at compile time
- `active_api_url`, `staging_url`, `production_url` fields to `DeviceConfig` struct — runtime URL selection based on `isLive` flag
- `getRuntimeDeviceConfig()` (`src/utils/deviceconfig.h`) — exposes current runtime config for the web UI
- `TESTING/` data directory on SD card — staging data written under `SENSORSDATA/TESTING/` when `isLive` is `false`
- `refreshLoggerPath()` / `updateLoggerPath()` — logger paths refreshed at runtime from `DeviceConfig.isLive` before file-logging and failed-payload resend operations
- `/device-config.json` web endpoint (`src/webserver/asyncserver.cpp`) — serves runtime device config

### Changed
- Full URLs (`src/global_configs.h`) — replaced `HOST_CFA` + `PATH_CFA` protocol/host/endpoint pattern with full `STAGING_URL` and `PRODUCTION_URL` constants
- `sendData()` and `readSendDelete()` now use `DeviceConfig.active_api_url` instead of hardcoded `CFA_URL`
- Safe string copy — all config macro assignments (`GSM_APN`, `WIFI_STA_SSID`, etc.) now use `strncpy` + explicit null termination
- `hasString()` helper — extended to accept booleans, integers, floats, and doubles in addition to strings
- MQTT telemetry loop — guarded behind `DeviceConfigState.isMQTTConfigured` to avoid work when MQTT is not set up
- `init_SD_loggers()` — refactored to use `snprintf` for structured path tree creation
- Comms init — switches preferred comm to GSM when WiFi SSID is empty and GSM is configured
- `SD_FOLDER_STRUCTURE.md` — updated to reflect new `TESTING/` directory

### Fixed
- HTTP response status type — changed `uint8_t statuscode` to `int` in `sendDataViaGSM()` to handle larger status codes
- Power saver config parsing — reads `config["powerSaver"]` as a proper boolean instead of comparing to string `"on"`
- Typo `useWifFi` → `useWiFi` in saved-config loading
- WiFi reconnection guard — `updateCommsPreference()` checks that `wifi_sta_ssid` is non-empty before attempting reconnect
- GSM network registration sequence — ensure all network modes are exhausted before giving up
- RTC get time — discard fetched time if device config time is not set
- Web UI power saver parsing — corrected boolean handling in frontend

## [v1.3.0](https://github.com/CodeForAfrica/sensors.AFRICA-ESP32-Quectel-Firmware/releases/tag/v1.3.0) 2026-05-05

### Added
- Device configuration and management via a web server
- WiFi connectivity support alongside GSM
- Device configuration overrides at runtime
- SD file downloads through the web server
- URL helper functions for request handling

### Changed
- Enhanced web server with device runtime flow updates
- Integrated web server UI as a submodule
- Refactored codebase for improved maintainability
- Archived unused codebases (kept for reference before deletion)

### Fixed
- Function redefinition conflicts
- Network registration loop causing hangs

## [v1.2.2](https://github.com/CodeForAfrica/sensors.AFRICA-ESP32-Quectel-Firmware/releases/tag/v1.2.2) 2026-03-11 — GSM Stable

### Fixed
- GSM communication stability improvements

## [v1.2.1](https://github.com/CodeForAfrica/sensors.AFRICA-ESP32-Quectel-Firmware/releases/tag/v1.2.1) 2025-06-13

### Improvements
- Device reboot feature after a specified period
- Reorganized codebase to improve readability

## [v1.2.0](https://github.com/CodeForAfrica/sensors.AFRICA-ESP32-Quectel-Firmware/releases/tag/v1.2.0) 2025-05-27

### Added 
- DHT support and logging
- C++17 support
  
### Improvements
- Templatize add_value2JSON_array to support both integers and floats
- Template type checking for CSV generator
- Extract sensor API_PIN from logged data

## 2025-05-26

### Added
- Custom board configuration file for `ESP32-S3-WROOM-1-N16R8`.

## 2025-04-24

### Added
- Generic structure for data loggers
- CSV data generation and logging

### Improvements
- JSON payload validation
- Format failed-to-send-payloads text file with `/r/n`
- Breakdown functionalities for logging and reading data.

## [v1.1.0](https://github.com/CodeForAfrica/sensors.AFRICA-ESP32-Quectel-Firmware/releases/tag/v1.1.0) 2025-04-14

### Added
- Sensor data values JSON object builder using Arduino JSON

### Changed
- Modified generate payload generator to use Arduino JSON
  
### Removed
- Deprecated add_Value2Json function






  
  