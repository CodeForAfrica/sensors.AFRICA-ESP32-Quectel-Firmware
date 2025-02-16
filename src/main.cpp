#include <Arduino.h>
#include "global_configs.h"
#include <SoftwareSerial.h>
#include "GSM_handler.h"

static const char HOST_CFA[] PROGMEM = "staging.api.sensors.africa";
static const char URL_CFA[] PROGMEM = "/v1/push-sensor-data/";
#define PMS_LED 3
#define PORT_CFA 80
bool send_now = false;
bool gsm_capable = false;

#define SOFTWARE_VERSION_STR "NRZ-2020-129"

const char data_first_part[] PROGMEM = "{\"software_version\": \"" SOFTWARE_VERSION_STR "\", \"sensordatavalues\":[";

constexpr unsigned SMALL_STR = 64 - 1;
constexpr unsigned MED_STR = 256 - 1;
constexpr unsigned LARGE_STR = 512 - 1;
constexpr unsigned XLARGE_STR = 1024 - 1;
bool airrohr_selftest_failed = false;
String esp_chipid;

#define RESERVE_STRING(name, size)    \
  String name((const char *)nullptr); \
  name.reserve(size)

#define msSince(timestamp_before) (act_milli - (timestamp_before))

template <typename T, std::size_t N>
constexpr std::size_t array_num_elements(const T (&)[N])
{
  return N;
}

int sampling_interval = 15 * 60 * 1000;
int sending_interval = 12 * 60 * 60 * 1000;
const unsigned long WARMUPTIME_SDS_MS = 15000;
int start_time = 0;
unsigned long act_milli;
unsigned long starttime = 0;
unsigned sending_intervall_ms = 145000;

int pms_pm1_sum = 0;
int pms_pm10_sum = 0;
int pms_pm25_sum = 0;
int pms_pm1_min = 20000;
int pms_pm10_min = 20000;
int pms_pm25_min = 20000;
int pms_pm1_max = 0;
int pms_pm10_max = 0;
int pms_pm25_max = 0;
int pms_val_count = 0;
float last_value_PMS_P0 = -1.0;
float last_value_PMS_P1 = -1.0;
float last_value_PMS_P2 = -1.0;

SoftwareSerial serialSDS;

// define serial interface pins for particle sensors
// Serial confusion: These definitions are based on SoftSerial
// TX (transmitting) pin on one side goes to RX (receiving) pin on other side
// SoftSerial RX PIN is D1 and goes to SDS TX
// SoftSerial TX PIN is D2 and goes to SDS RX
#define PM_SERIAL_RX D1
#define PM_SERIAL_TX D2

bool is_PMS_running = true;

void setup()
{
#if defined(ESP8266)
  Serial.begin(9600); // Output to Serial at 9600 baud
#endif
#if defined(ESP32)
  Serial.begin(115200); // Output to Serial at 15200 baud
#endif
#if defined(ESP8266)
  serialSDS.begin(9600, SWSERIAL_8N1, PM_SERIAL_RX, PM_SERIAL_TX);
#endif
#if defined(ESP32)
  serialSDS.begin(9600, SERIAL_8N1, PM_SERIAL_RX, PM_SERIAL_TX);
#endif
  serialSDS.enableIntTx(true);
  serialSDS.setTimeout((12 * 9 * 1000) / 9600);

  Wire.begin(I2C_PIN_SDA, I2C_PIN_SCL);
  pinMode(PMS_LED, OUTPUT);

#if defined(ESP8266)
  esp_chipid = std::move(String(ESP.getChipId()));
#endif
#if defined(ESP32)
  uint64_t chipid_num;
  chipid_num = ESP.getEfuseMac();
  esp_chipid = String((uint16_t)(chipid_num >> 32), HEX);
  esp_chipid += String((uint32_t)chipid_num, HEX);
#endif

  if ((airrohr_selftest_failed = !ESP.checkFlashConfig(true) /* after 2.7.0 update: || !ESP.checkFlashCRC() */))
  {
    Serial.println("ERROR: SELF TEST FAILED!");
    // SOFTWARE_VERSION += F("-STF");
  }

  Serial.println("\nChipId: " + esp_chipid);

  if (gsm_capable)
  {

    Serial.println("Attempting to setup GSM connection");

    pinMode(QUECTEL_PWR_KEY, OUTPUT);

    if (!GSM_init(fonaSerial))
    {
      Serial.println("GSM not fully configured");
      Serial.print("Failure point: ");
      Serial.println(GSM_INIT_ERROR);
      Serial.println();
    }
  }
  // if (!GPRS_CONNECTED)
  // {
  // 	connectWifi();
  // }
  else
  {
    connectWifi();
  }

  // setupNetworkTime();

  powerOnTestSensors();

  delay(50);

  // sometimes parallel sending data and web page will stop nodemcu, watchdogtimer set to 120 seconds
#if defined(ESP8266)
  wdt_disable();
#if defined(NDEBUG)
  wdt_enable(120000);
#endif
#endif

  starttime = millis(); // store the start time
}

void loop()
{
  // put your main code here, to run repeatedly:

  String result_PMS;
}

/*****************************************************************
 * add value to json string                                  *
 *****************************************************************/
static void add_Value2Json(String &res, const __FlashStringHelper *type, const String &value)
{
  RESERVE_STRING(s, SMALL_STR);

  s = F("{\"value_type\":\"{t}\",\"value\":\"{v}\"},");
  s.replace("{t}", String(type));
  s.replace("{v}", value);
  res += s;
}

static void add_Value2Json(String &res, const __FlashStringHelper *type, const __FlashStringHelper *debug_type, const float &value)
{
  Serial.print(FPSTR(debug_type));
  Serial.println(value);
  add_Value2Json(res, type, String(value));
}

enum class PmSensorCmd
{
  Start,
  Stop,
  ContinuousMode
};

const unsigned long SAMPLETIME_SDS_MS = 1000;  // time between two measurements of the SDS011, PMSx003, Honeywell PM sensor
const unsigned long WARMUPTIME_SDS_MS = 15000; // time needed to "warm up" the sensor before we can take the first measurement
const unsigned long READINGTIME_SDS_MS = 5000; // how long we read data from the PM sensors

/*****************************************************************
 * send Plantower PMS sensor command start, stop, cont. mode     *
 *****************************************************************/
static bool PMS_cmd(PmSensorCmd cmd)
{
  static constexpr uint8_t start_cmd[] PROGMEM = {
      0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
  static constexpr uint8_t stop_cmd[] PROGMEM = {
      0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};
  static constexpr uint8_t continuous_mode_cmd[] PROGMEM = {
      0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71};
  constexpr uint8_t cmd_len = array_num_elements(start_cmd);

  uint8_t buf[cmd_len];
  switch (cmd)
  {
  case PmSensorCmd::Start:
    memcpy_P(buf, start_cmd, cmd_len);
    break;
  case PmSensorCmd::Stop:
    memcpy_P(buf, stop_cmd, cmd_len);
    break;
  case PmSensorCmd::ContinuousMode:
    memcpy_P(buf, continuous_mode_cmd, cmd_len);
    break;
  }
  serialSDS.write(buf, cmd_len);
  return cmd != PmSensorCmd::Stop;
}

/*****************************************************************
 * read Plantronic PM sensor sensor values                       *
 *****************************************************************/
static void fetchSensorPMS(String &s)
{
  char buffer;
  int value;
  int len = 0;
  int pm1_serial = 0;
  int pm10_serial = 0;
  int pm25_serial = 0;
  int checksum_is = 0;
  int checksum_should = 0;
  bool checksum_ok = false;
  int frame_len = 24; // min. frame length

  Serial.print("Start reading PMS503 sensor: ");

  if (msSince(starttime) < (sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
  {
    if (is_PMS_running)
    {
      is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
    }
  }
  else
  {
    if (!is_PMS_running)
    {
      is_PMS_running = PMS_cmd(PmSensorCmd::Start);
    }

    while (serialSDS.available() > 0)
    {
      buffer = serialSDS.read();
      Serial.println(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .");
      //			"aa" = 170, "ab" = 171, "c0" = 192
      value = int(buffer);
      switch (len)
      {
      case 0:
        if (value != 66)
        {
          len = -1;
        };
        break;
      case 1:
        if (value != 77)
        {
          len = -1;
        };
        break;
      case 2:
        checksum_is = value;
        break;
      case 3:
        frame_len = value + 4;
        break;
      case 10:
        pm1_serial += (value << 8);
        break;
      case 11:
        pm1_serial += value;
        break;
      case 12:
        pm25_serial = (value << 8);
        break;
      case 13:
        pm25_serial += value;
        break;
      case 14:
        pm10_serial = (value << 8);
        break;
      case 15:
        pm10_serial += value;
        break;
      case 22:
        if (frame_len == 24)
        {
          checksum_should = (value << 8);
        };
        break;
      case 23:
        if (frame_len == 24)
        {
          checksum_should += value;
        };
        break;
      case 30:
        checksum_should = (value << 8);
        break;
      case 31:
        checksum_should += value;
        break;
      }
      if ((len > 2) && (len < (frame_len - 2)))
      {
        checksum_is += value;
      }
      len++;
      if (len == frame_len)
      {
        Serial.print("Checksum is: ");
        Serial.print(String(checksum_is + 143));
        Serial.print("\tChecksum should be: ");
        Serial.println(String(checksum_should));

        if (checksum_should == (checksum_is + 143))
        {
          checksum_ok = true;
        }
        else
        {
          len = 0;
        };
        if (checksum_ok && (msSince(starttime) > (sending_intervall_ms - READINGTIME_SDS_MS)))
        {
          if ((!isnan(pm1_serial)) && (!isnan(pm10_serial)) && (!isnan(pm25_serial)))
          {
            pms_pm1_sum += pm1_serial;
            pms_pm10_sum += pm10_serial;
            pms_pm25_sum += pm25_serial;
            if (pms_pm1_min > pm1_serial)
            {
              pms_pm1_min = pm1_serial;
            }
            if (pms_pm1_max < pm1_serial)
            {
              pms_pm1_max = pm1_serial;
            }
            if (pms_pm25_min > pm25_serial)
            {
              pms_pm25_min = pm25_serial;
            }
            if (pms_pm25_max < pm25_serial)
            {
              pms_pm25_max = pm25_serial;
            }
            if (pms_pm10_min > pm10_serial)
            {
              pms_pm10_min = pm10_serial;
            }
            if (pms_pm10_max < pm10_serial)
            {
              pms_pm10_max = pm10_serial;
            }

            Serial.print("PM1 (sec.): ");
            Serial.println(String(pm1_serial));
            Serial.print("PM2.5 (sec.): ");
            Serial.println(String(pm25_serial));
            Serial.print("PM10 (sec.) : ");
            Serial.println(String(pm10_serial));

            pms_val_count++;
          }
          len = 0;
          checksum_ok = false;
          pm1_serial = 0;
          pm10_serial = 0;
          pm25_serial = 0;
          checksum_is = 0;
        }
      }
      yield();
    }
  }
  if (send_now)
  {
    last_value_PMS_P0 = -1;
    last_value_PMS_P1 = -1;
    last_value_PMS_P2 = -1;
    if (pms_val_count > 2)
    {
      pms_pm1_sum = pms_pm1_sum - pms_pm1_min - pms_pm1_max;
      pms_pm10_sum = pms_pm10_sum - pms_pm10_min - pms_pm10_max;
      pms_pm25_sum = pms_pm25_sum - pms_pm25_min - pms_pm25_max;
      pms_val_count = pms_val_count - 2;
    }
    if (pms_val_count > 0)
    {
      last_value_PMS_P0 = float(pms_pm1_sum) / float(pms_val_count);
      last_value_PMS_P1 = float(pms_pm10_sum) / float(pms_val_count);
      last_value_PMS_P2 = float(pms_pm25_sum) / float(pms_val_count);
      add_Value2Json(s, F("PMS_P0"), F("PM1:   "), last_value_PMS_P0);
      add_Value2Json(s, F("PMS_P1"), F("PM10:  "), last_value_PMS_P1);
      add_Value2Json(s, F("PMS_P2"), F("PM2.5: "), last_value_PMS_P2);
      Serial.println("----");
    }
    pms_pm1_sum = 0;
    pms_pm10_sum = 0;
    pms_pm25_sum = 0;
    pms_val_count = 0;
    pms_pm1_max = 0;
    pms_pm1_min = 20000;
    pms_pm10_max = 0;
    pms_pm10_min = 20000;
    pms_pm25_max = 0;
    pms_pm25_min = 20000;
    if (sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS))
    {
      is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
    }
    digitalWrite(PMS_LED, HIGH);
    delay(5000);
    digitalWrite(PMS_LED, LOW);
  }

  Serial.println("End reading PMS503 sensor");
}

static void powerOnTestSensors()
{

  Serial.println("Read PMS(1,3,5,6,7)003...");
  PMS_cmd(PmSensorCmd::Start);
  delay(100);
  PMS_cmd(PmSensorCmd::ContinuousMode);
  delay(100);
  Serial.println("Stopping PMS...");
  is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
}

/*****************************************************************
 * send data to rest api                                         *
 *****************************************************************/
static unsigned long sendData(const String &data, const int pin, const char *host, const char *url)
{

  unsigned long start_send = millis();
  String contentType = "application/json";
  int result = 0;
  int port;

  if (cfg::gsm_capable)
  {

    if (!GPRS_CONNECTED)
    {
      // if (!GPRS_init())
      // {

      // 	if (SIM_USABLE)
      // 	{
      // 		GSM_soft_reset();
      // 	}
      // }
      if (!GPRS_init())
      {

        GPRS_INIT_FAIL_COUNT += 1;
        Serial.print("GPRS INIT FAIL COUNT: ");
        Serial.println(GPRS_INIT_FAIL_COUNT);
        if (GPRS_INIT_FAIL_COUNT == 5)
        { //! RESET COUNTER
          GPRS_INIT_FAIL_COUNT = 0;
          GSM_soft_reset();
          GSM_init(fonaSerial);
        }
      }
    }
    if (GPRS_CONNECTED)
    {
      int retry_count = 0;
      uint16_t statuscode;
      int16_t length;

      String gprs_request_head = F("X-PIN: ");
      gprs_request_head += String(pin) + "\\r\\n";
      gprs_request_head += F("X-Sensor: esp8266-");
      gprs_request_head += esp_chipid;

#ifdef QUECTEL
      String Quectel_headers[3];
      Quectel_headers[0] = "X-PIN: " + String(pin);
      Quectel_headers[1] = "X-Sensor: esp8266-" + esp_chipid;
      // Quectel_headers[1] = "X-Sensor: esp8266-quectel-test";       // testing node, comment and insert desired testing node ID
      Quectel_headers[2] = "Content-Type: " + contentType; // 30

      int header_size = sizeof(Quectel_headers) / sizeof(Quectel_headers[0]);

#endif

      const char *data_copy = data.c_str();
      char gprs_data[strlen(data_copy)];
      strcpy(gprs_data, data_copy);

      String post_url = String(s_Host);
      post_url += String(s_url);
      const char *url_copy = post_url.c_str();
      char gprs_url[strlen(url_copy)];
      strcpy(gprs_url, url_copy);

      Serial.println("POST URL  " + String(gprs_url));

      debug_out(F("Sending data via gsm"), DEBUG_MIN_INFO);
      debug_out(F("http://"), DEBUG_MIN_INFO);
      debug_out(gprs_url, DEBUG_MIN_INFO);
      debug_out(gprs_data, DEBUG_MIN_INFO);
      Serial.println("GPRS REQUEST HEAD:");
      Serial.println(gprs_request_head);
      Serial.println();
      flushSerial();
      debug_out(F("## Sending via gsm\n\n"), DEBUG_MIN_INFO);

#ifdef QUECTEL
      QUECTEL_POST((char *)gprs_url, Quectel_headers, header_size, data, data.length());
      // ToDo: close HTTP session/ PDP context
#else
      if (!fona.HTTP_POST_start((char *)gprs_url, F("application/json"), gprs_request_head, (uint8_t *)gprs_data, strlen(gprs_data), &statuscode, (uint16_t *)&length))
      {
        Serial.print("Failed with status code: ");
        Serial.println(String(statuscode)); // !ERROR not handled correctly: POST in most cases is successul but the status code !=200
        disableGPRS();
        return 0;
      }
      while (length > 0)
      {
        while (fona.available())
        {
          char c = fona.read();
          Serial.write(c);
          length--;
          if (!length)
            break;
        }
      }
      debug_out(F("\n\n## End sending via gsm \n\n"), DEBUG_MIN_INFO);
      fona.HTTP_POST_end();
      disableGPRS();
#endif
    }
  }

  wdt_reset();
  yield();
  return millis() - start_send;
}

/*****************************************************************
 * send single sensor data to sensors.AFRICA api                  *
 *****************************************************************/
static unsigned long sendCFA(const String &data, const int pin, const __FlashStringHelper *sensorname, const char *replace_str)
{
  unsigned long sum_send_time = 0;

  if (cfg::send2cfa && data.length())
  {
    RESERVE_STRING(data_CFA, LARGE_STR);
    data_CFA = FPSTR(data_first_part);

    Serial.println("## Sending to sensors.AFRICA - " + String(sensorname));
    data_CFA += data;
    data_CFA.remove(data_CFA.length() - 1);
    data_CFA.replace(replace_str, emptyString);
    data_CFA += "]}";
    Serial.println(data_CFA);

    sum_send_time = sendData(data_CFA, pin, HOST_CFA, URL_CFA);
  }

  return sum_send_time;
}