# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v1.2.1](https://github.com/gideonmaina/sensors_power_saver/releases/tag/v1.2.1) 2025-06-13

### Improvements
- Device reboot feature after a specified period
- Reorganized codebase to improve readability

## [v1.2.0](https://github.com/gideonmaina/sensors_power_saver/releases/tag/v1.2.0) 2025-05-27

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

## [Unreleased](https://github.com/gideonmaina/sensors_power_saver/releases/tag/v1.1.0) v.1.1.0 (2025-04-14)

### Added
 - sensor data values JSON object builder using Arduino JSON.

 ### Changed
 - Modified generate payload generator to use Arduino JSON. 
  
### Removed
 - Deprecated add_Value2Json function






  
  