#ifndef GLOBAL_CONFIGS_H
#define GLOBAL_CONFIGS_H

static const char STAGING_URL[] = "http://sensors-api.dev.codeforafrica.org/v1/push-sensor-data/";
static const char PRODUCTION_URL[] = "https://api.sensors.africa/v1/push-sensor-data/";

#define PORT_CFA 80
#define IS_LIVE false

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

#define MQTT_BASE_TOPIC "devices/nodes/telemetry"
#define MQTT_BROKER "" // server must be set to enable MQTT telemetry
#define MQTT_PORT 1883
#define MQTT_USERNAME "" // set to enable MQTT authentication
#define MQTT_PASSWORD "" // set to enable MQTT authentication
#define MQTT_CLIENT_ID 5
#define MQTT_SUBSCRIBE_TOPIC "devices/nodes/configuration"

// ---------------------------------------------------------------------------
// Home Assistant integration (MQTT Discovery over WiFi)
// Set HA_ENABLE to 1 and point HA_MQTT_BROKER at your Home Assistant broker
// (e.g. the IP address of a Raspberry Pi running Home Assistant).
// Full setup instructions: see HOME_ASSISTANT.md
// ---------------------------------------------------------------------------
#define HA_ENABLE 0
#define HA_MQTT_BROKER "" // e.g. "192.168.1.50" or "homeassistant.local"
#define HA_MQTT_PORT 1883
#define HA_MQTT_USERNAME "" // set if your HA MQTT broker requires authentication
#define HA_MQTT_PASSWORD "" // set if your HA MQTT broker requires authentication
#define HA_DISCOVERY_PREFIX "homeassistant"
#define HA_DEVICE_NAME "Sensors Africa Node"
#define HA_DEVICE_MANUFACTURER "Code for Africa"
#define HA_DEVICE_MODEL "ESP32-S3 Sensor Node"
#endif