#ifndef GLOBAL_CONFIGS_H
#define GLOBAL_CONFIGS_H

static const char HOST_CFA[] = "staging.api.sensors.africa";
static const char URL_CFA[] = "/v1/push-sensor-data/";
#define PORT_CFA 80

static const char SENSOR_PREFIX[] = "ESP32-";

// user predefined device configurations
#define POWER_SAVING_MODE 1

static bool gsm_capable = true;
static bool use_wifi = true;
static bool use_gsm = true;

enum CommunicationPriority
{
    WIFI,
    GSM,
};

#define GSM_DEBUG true

#define QUECTEL EC200CN

#define PMS_API_PIN 1
#define DHT_API_PIN 7

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
static int SD_SCK = 38;
static int SD_MISO = 41;
static int SD_MOSI = 40;
static int SD_CS = 39;

// #if defined(ESP32)
// define pin for one wire sensors
#define ONEWIRE_PIN 36 // PIN 29
#define DHTTYPE 22     // DHT22 sensor type

// define pins for status LEDs
#define PMS_LED 35 // PIN 28
#define DHT_LED 37 // 30
                   // endif

#endif