#include <Arduino.h>
#include "global_configs.h"
#include <SoftwareSerial.h>
#include "GSM_handler.h"
#include "SD_handler.h"
#include <TimeLib.h>

bool send_now = false;

bool airrohr_selftest_failed = false;
String esp_chipid;
String my_espchid;
char time_buff[23];

// Define the maximum number of sensor data and the maximum length of each string
const int MAX_STRINGS = 48; // 12*60/15 units of minutes
// const int MAX_STRING_LENGTH = 100;
// String sensor_data[20] = {};
uint8_t sensor_data_log_count = 0;
String sensor_data[MAX_STRINGS]; // ? Change this to char array

#define RESERVE_STRING(name, size)    \
  String name((const char *)nullptr); \
  name.reserve(size)

#define msSince(timestamp_before) (act_milli - (timestamp_before))

template <typename T, std::size_t N>
constexpr std::size_t array_num_elements(const T (&)[N])
{
  return N;
}

int sampling_interval = 30000;
int sending_interval = 12 * 60 * 60 * 1000;
unsigned long last_read_pms = 0;
const unsigned long WARMUPTIME_SDS_MS = 15000; // time needed to "warm up" the sensor before we can take the first measurement
const unsigned long SAMPLETIME_SDS_MS = 1000;  // time between two measurements of the SDS011, PMSx003, Honeywell PM sensor
const unsigned long READINGTIME_SDS_MS = 5000; // how long we read data from the PM sensors
int start_time = 0;
unsigned long act_milli;
unsigned long starttime = 0;
unsigned sending_intervall_ms = 1 * 60 * 1000; // 145000;
unsigned long starttime_SDS;
unsigned long sending_time = 0;
unsigned long count_sends = 0;

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

#if defined(ESP8266)
SoftwareSerial serialSDS;
#define SENSOR_PREFIX "esp8266-"
#endif
#if defined(ESP32)
#include <HardwareSerial.h>
#define serialSDS (Serial2)
// SoftwareSerial serialSDS(PM_SERIAL_RX, PM_SERIAL_TX);
#define SENSOR_PREFIX "esp32-"
#endif

bool is_PMS_running = true;
bool SD_MOUNT = false;
bool SD_Attached = false;

char SENSORS_DATA_DIR[20] = "/SENSORSDATA";
char SENSORS_DATA_PATH[40] = "/SENSORSDATA/sensordatalog.txt";

// Function declarations
static void fetchSensorPMS(String &s);
static void powerOnTestSensors();
static unsigned long sendData(const String &data, const int pin, const char *host, const char *url);
static unsigned long sendCFA(const String &data, const int pin, const __FlashStringHelper *sensorname, const char *replace_str);
String extractDateTime(String datetimeStr);
String formatDateTime(time_t t, String timezone);

void setup()
{
  delay(5000);
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
  Serial.println("Serial SDS begin");
  // serialSDS.begin(9600);
  serialSDS.begin(9600, SERIAL_8N1, PM_SERIAL_RX, PM_SERIAL_TX);
  Serial.println("Serial SDS already began");
#endif
  // serialSDS.enableIntTx(true);
  serialSDS.setTimeout((12 * 9 * 1000) / 9600);
  pinMode(PMS_LED, OUTPUT);

#if defined(ESP8266)
  esp_chipid = std::move(String(ESP.getChipId()));
#endif
#if defined(ESP32)
  uint64_t chipid_num;
  chipid_num = ESP.getEfuseMac();
  esp_chipid = String((uint16_t)(chipid_num >> 32), HEX);
  esp_chipid += String((uint32_t)chipid_num, HEX);
  my_espchid = String((uint64_t)chipid_num, HEX);
#endif

  // if ((airrohr_selftest_failed = !ESP.checkFlashConfig(true) /* after 2.7.0 update: || !ESP.checkFlashCRC() */))
  // {
  //   Serial.println("ERROR: SELF TEST FAILED!");
  //   // SOFTWARE_VERSION += F("-STF");
  // }

  Serial.println("\nChipId: " + esp_chipid);
  Serial.println("My ChipId: " + my_espchid);

  // SD Init

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS))
  {

    Serial.println("Card Mount Failed");
    SD_MOUNT = false;
  }
  else
  {
    SD_MOUNT = true;
  }
  SD_Attached = SDattached();

  if (SD_Attached)
  {
    createDir(SD, SENSORS_DATA_DIR);
  }

  if (gsm_capable)
  {
    fonaSS.begin(115200);
    if (!validate_GSM_serial_communication())
    {
      Serial.println("GSM not detected. Restarting ESP");
      // ESP.sleep();
      delay(2000);
      ESP.restart();
    }

    if (!GSM_init()) // ! Why hang here after ESP restart?
    {
      Serial.println("GSM not fully configured");
      Serial.print("Failure point: ");
      Serial.println(error_buffer);
      Serial.println();
      // delay(10000);
      // ESP.restart();
    }

    // while (!fona.getTime(time_buff, 23))
    // {
    //   Serial.println("Failed to fetch time from network");
    //   delay(1000);
    // };
    // Serial.println("Time: " + String(time_buff));
  }

  else
  {
    // connectWifi();
    Serial.println("Support for other network connections not yet implemented");
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

  starttime = starttime_SDS = millis(); // store the start time
}

void loop()
{
  // put your main code here, to run repeatedly:

  static String result_PMS;
  unsigned sum_send_time = 0;
  act_milli = millis();

  send_now = msSince(starttime) > sending_intervall_ms;

  // if ((msSince(starttime_SDS) > SAMPLETIME_SDS_MS) || send_now)
  // {
  //   starttime_SDS = act_milli;

  //   fetchSensorPMS(result_PMS);
  //   Serial.print("PMS_Readings: ");
  //   Serial.println(result_PMS);
  // }

  if (act_milli - last_read_pms > sampling_interval)
  {
    starttime_SDS = act_milli;
    Serial.println("Act_milli: " + String(act_milli));
    result_PMS = "";
    fetchSensorPMS(result_PMS);
    char read_time[23];
    fona.getTime(read_time, 23);
    Serial.print("PMS_Readings: ");
    Serial.println(result_PMS);
    last_read_pms = millis();
    Serial.println("Last read PMS: " + String(last_read_pms));

    String datetime = "";
    if (String(read_time) != "")
    {
      datetime = extractDateTime(String(read_time));
    }
    if (datetime == "")
    {
      Serial.println("Datetime is empty...discarding data point");
    }
    else
    {
      if (sensor_data_log_count < MAX_STRINGS)
      {

        // Save data to array
        RESERVE_STRING(data, LARGE_STR);
        data = FPSTR(data_first_part);
        data += result_PMS;
        if ((unsigned)(data.lastIndexOf(',') + 1) == data.length())
        {
          data.remove(data.length() - 1);
        }
        data += "],";

        data += "\"timestamp\":\"";
        data += String(datetime);

        if ((unsigned)(data.lastIndexOf(',') + 1) == data.length())
        {
          data.remove(data.length() - 1);
        }
        data += "\"}";
        sensor_data[sensor_data_log_count] = data;
        Serial.println("Sensor data:" + sensor_data[sensor_data_log_count]);
        sensor_data_log_count++;
      }
      else
      {
        Serial.println("Sensor data log count exceeded");
        // Save data to SD and empty array;
      }
    }
  }

  if (send_now)
  {

    // save data to SD
    for (int i = 0; i < sensor_data_log_count; i++)
    {
      if (sensor_data[i] != "")
      {

        const char *data_to_append = sensor_data[i].c_str();
        char fmt_data[strlen(data_to_append) + 2];
        strcpy(fmt_data, data_to_append);
        strcat(fmt_data, "\n");
        appendFile(SD, SENSORS_DATA_PATH, fmt_data);
      }
    }

    // Read data from array and send to server
    for (int i = 0; i < sensor_data_log_count; i++)
    {
      sum_send_time += sendData(sensor_data[i], PMS_API_PIN, HOST_CFA, URL_CFA);
      // Remove data from array
      sensor_data[i] = "";
    }

    Serial.println("Time for Sending (ms): " + String(sum_send_time));

    // Resetting for next sampling
    sum_send_time = 0;
    sensor_data_log_count = 0;

    count_sends++;
    Serial.println("Sent data counts: " + count_sends);
    starttime = millis(); // store the start time
  }

  //! Debugging. Comment out during upload

  readFile(SD, SENSORS_DATA_PATH);
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

  // if (msSince(starttime) < (sending_intervall_ms - (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS)))
  // {
  //   if (is_PMS_running)
  //   {
  //     Serial.println("PMS cmd stop");
  //     is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
  //   }
  // }
  // else
  // {
  if (!is_PMS_running)
  {
    Serial.println("PMS cmd start");
    is_PMS_running = PMS_cmd(PmSensorCmd::Start);
    delay(WARMUPTIME_SDS_MS + READINGTIME_SDS_MS);
  }
  Serial.println("Checking for data...");
  while (serialSDS.available() > 0)
  {
    buffer = serialSDS.read();
    // Serial.println(String(len) + " - " + String(buffer, DEC) + " - " + String(buffer, HEX) + " - " + int(buffer) + " .");
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
      // Serial.print("Checksum is: ");
      // Serial.print(String(checksum_is + 143));
      // Serial.print("\tChecksum should be: ");
      // Serial.println(String(checksum_should));

      if (checksum_should == (checksum_is + 143))
      {
        checksum_ok = true;
      }
      else
      {
        len = 0;
      };
      // if (checksum_ok && (msSince(starttime) > (sending_intervall_ms - READINGTIME_SDS_MS)))
      if (checksum_ok)
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

          // Serial.print("PM1 (sec.): ");
          // Serial.println(String(pm1_serial));
          // Serial.print("PM2.5 (sec.): ");
          // Serial.println(String(pm25_serial));
          // Serial.print("PM10 (sec.) : ");
          // Serial.println(String(pm10_serial));

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
    // yield();
  }

  is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
  add_Value2Json(s, F("P0"), F("PM1:   "), pm1_serial);
  add_Value2Json(s, F("P1"), F("PM10:  "), pm10_serial);
  add_Value2Json(s, F("P2"), F("PM2.5: "), pm25_serial);
  digitalWrite(PMS_LED, HIGH);
  delay(5000);
  digitalWrite(PMS_LED, LOW);
  //}
  // if (send_now)
  // {
  //   last_value_PMS_P0 = -1;
  //   last_value_PMS_P1 = -1;
  //   last_value_PMS_P2 = -1;
  //   if (pms_val_count > 2)
  //   {
  //     pms_pm1_sum = pms_pm1_sum - pms_pm1_min - pms_pm1_max;
  //     pms_pm10_sum = pms_pm10_sum - pms_pm10_min - pms_pm10_max;
  //     pms_pm25_sum = pms_pm25_sum - pms_pm25_min - pms_pm25_max;
  //     pms_val_count = pms_val_count - 2;
  //   }
  //   if (pms_val_count > 0)
  //   {
  //     last_value_PMS_P0 = float(pms_pm1_sum) / float(pms_val_count);
  //     last_value_PMS_P1 = float(pms_pm10_sum) / float(pms_val_count);
  //     last_value_PMS_P2 = float(pms_pm25_sum) / float(pms_val_count);
  //     add_Value2Json(s, F("PMS_P0"), F("PM1:   "), last_value_PMS_P0);
  //     add_Value2Json(s, F("PMS_P1"), F("PM10:  "), last_value_PMS_P1);
  //     add_Value2Json(s, F("PMS_P2"), F("PM2.5: "), last_value_PMS_P2);
  //     Serial.println("----");
  //   }
  //   pms_pm1_sum = 0;
  //   pms_pm10_sum = 0;
  //   pms_pm25_sum = 0;
  //   pms_val_count = 0;
  //   pms_pm1_max = 0;
  //   pms_pm1_min = 20000;
  //   pms_pm10_max = 0;
  //   pms_pm10_min = 20000;
  //   pms_pm25_max = 0;
  //   pms_pm25_min = 20000;
  //   if (sending_intervall_ms > (WARMUPTIME_SDS_MS + READINGTIME_SDS_MS))
  //   {
  //     is_PMS_running = PMS_cmd(PmSensorCmd::Stop);
  //   }
  //   digitalWrite(PMS_LED, HIGH);
  //   delay(5000);
  //   digitalWrite(PMS_LED, LOW);
  // }

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

  if (gsm_capable)
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
        if (GPRS_INIT_FAIL_COUNT == 3)
        { //! RESET COUNTER
          GPRS_INIT_FAIL_COUNT = 0;
          GSM_soft_reset();
          // GSM_init();
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
      gprs_request_head += F("X-Sensor: ");
      gprs_request_head += SENSOR_PREFIX;
      gprs_request_head += esp_chipid;

#ifdef QUECTEL
      String Quectel_headers[3];
      Quectel_headers[0] = "X-PIN: " + String(pin);
      Quectel_headers[1] = "X-Sensor: ";
      Quectel_headers[1] += SENSOR_PREFIX;
      Quectel_headers[1] += esp_chipid;
      // Quectel_headers[1] = "X-Sensor: esp8266-quectel-test";       // testing node, comment and insert desired testing node ID
      Quectel_headers[2] = "Content-Type: " + contentType; // 30

      int header_size = sizeof(Quectel_headers) / sizeof(Quectel_headers[0]);

#endif

      const char *data_copy = data.c_str();
      char gprs_data[strlen(data_copy)];
      strcpy(gprs_data, data_copy);

      String post_url = String(host);
      post_url += String(url);
      const char *url_copy = post_url.c_str();
      char gprs_url[strlen(url_copy)];
      strcpy(gprs_url, url_copy);

      // Serial.println("POST URL  " + String(gprs_url));

      // Serial.print("Sending data via gsm");
      // Serial.print("\thttp://");
      // Serial.println(gprs_url);
      // Serial.println(gprs_data);
      // Serial.println("GPRS REQUEST HEAD:");
      // Serial.println(gprs_request_head);
      // Serial.println();
      // flushSerial();

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
      Serial.println("\n\n## End sending via gsm \n\n");
      fona.HTTP_POST_end();
      disableGPRS();
#endif
    }
  }

#if defined(ESP8266)
  wdt_reset();
#endif
  yield();
  return millis() - start_send;
}

/*****************************************************************
 * send single sensor data to sensors.AFRICA api                  *
 *****************************************************************/
static unsigned long sendCFA(const String &data, const int pin, const __FlashStringHelper *sensorname, const char *replace_str)
{
  unsigned long sum_send_time = 0;

  if (data.length())
  {
    RESERVE_STRING(data_CFA, LARGE_STR);
    data_CFA = FPSTR(data_first_part);

    Serial.println("## Sending to sensors.AFRICA - " + String(sensorname));
    data_CFA += data;
    data_CFA.remove(data_CFA.length() - 1);
    data_CFA.replace(replace_str, emptyString);
    data_CFA += "]}";
    Serial.print("\nCFA Data to send: ");
    Serial.println(data_CFA);

    sum_send_time = sendData(data_CFA, pin, HOST_CFA, URL_CFA);
  }

  return sum_send_time;
}

String extractDateTime(String datetimeStr)
{

  Serial.println("Received date string: " + datetimeStr); //! format looks like "25/02/24,05:55:53+00" including the quotes!

  // remove the first and last character of the string (")
  datetimeStr = datetimeStr.substring(1, datetimeStr.length() - 1);

  // Parse the datetime string

  int day = datetimeStr.substring(0, 2).toInt();
  int month = datetimeStr.substring(3, 5).toInt();
  int year = datetimeStr.substring(6, 8).toInt();
  int hour = datetimeStr.substring(9, 11).toInt();
  int minute = datetimeStr.substring(12, 14).toInt();
  int second = datetimeStr.substring(15, 17).toInt();
  String timezone = datetimeStr.substring(17); // +00

  Serial.println("Day: " + String(day));
  Serial.println("Month: " + String(month));
  Serial.println("Year: " + String(year));
  Serial.println("Hour: " + String(hour));
  Serial.println("Minute: " + String(minute));
  Serial.println("Second: " + String(second));

  // Adjust year for TimeLib (TimeLib expects years since 1970)
  year += 2000; // Assuming 24 is 2024
  year -= 1970;

  // Create a tmElements_t struct
  tmElements_t tm;
  tm.Second = second;
  tm.Minute = minute;
  tm.Hour = hour;
  tm.Day = day;
  tm.Month = month;
  tm.Year = year;

  // Create a time_t value
  time_t t = makeTime(tm);

  // Format the time_t value to YYYY-MM-DDThh:mm:ss+HH:MM
  String formattedDateTime = formatDateTime(t, timezone);
  Serial.print("Formatted DateTime: ");
  Serial.println(formattedDateTime);
  return formattedDateTime;
}
String formatDateTime(time_t t, String timezone)
{
  String yearStr = String(year(t)); // Adjust year back to 20xx
  String monthStr = String(month(t));
  String dayStr = String(day(t));
  String hourStr = String(hour(t));
  String minuteStr = String(minute(t));
  String secondStr = String(second(t));

  // Pad with leading zeros if necessary
  if (monthStr.length() == 1)
    monthStr = "0" + monthStr;
  if (dayStr.length() == 1)
    dayStr = "0" + dayStr;
  if (hourStr.length() == 1)
    hourStr = "0" + hourStr;
  if (minuteStr.length() == 1)
    minuteStr = "0" + minuteStr;
  if (secondStr.length() == 1)
    secondStr = "0" + secondStr;

  return yearStr + "-" + monthStr + "-" + dayStr + "T" + hourStr + ":" + minuteStr + ":" + secondStr + timezone;
}