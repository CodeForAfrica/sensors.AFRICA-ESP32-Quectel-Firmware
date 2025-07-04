
#ifndef GLOBAL_CONFIGS_H
#define GLOBAL_CONFIGS_H

static const char HOST_CFA[] = "staging.api.sensors.africa";
static const char URL_CFA[] = "/v1/push-sensor-data/";
#define PORT_CFA 80

static const char SENSOR_PREFIX[] = "ESP32-";

constexpr unsigned SMALL_STR = 64 - 1;
constexpr unsigned MED_STR = 256 - 1;
constexpr unsigned LARGE_STR = 512 - 1;
constexpr unsigned XLARGE_STR = 1024 - 1;

#define POWER_SAVING_MODE 1

bool gsm_capable = true;
#define GSM_DEBUG

#define QUECTEL EC200CN

#define PMS_API_PIN 1

// PIN DEFINITIONS
#define MCU_RXD 17
#define MCU_TXD 18
#define QUECTEL_PWR_KEY 16
#define GSM_RST_PIN 42 // PIN 35

#define GSM_PIN ""

#define PM_SERIAL_RX 21 // PIN 26
#define PM_SERIAL_TX 45 // PIN 23

// SD CARD
#define REASSIGN_PINS 1
int SD_SCK = 38;
int SD_MISO = 41;
int SD_MOSI = 40;
int SD_CS = 39;

// #if defined(ESP32)
// define pin for one wire sensors
#define ONEWIRE_PIN 36 // PIN 29
#define DHTTYPE 22     // DHT22 sensor type

// define pins for status LEDs
#define PMS_LED 35 // PIN 28
#define DHT_LED 37 // 30
                   // endif

#endif