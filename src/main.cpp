/**
 * @file main.cpp
 * @brief Implementation of an ESP32-based sensor data logger and transmitter.
 *
 * This program is designed to collect data from sensors (e.g., PMS5003), log the data in memory and on an SD card,
 * and transmit the data to a remote server using GSM/GPRS. The program supports JSON and CSV data formats for logging
 * and transmission. It also handles network time synchronization and manages failed data transmissions by retrying
 * them later.
 *
 * @details
 * - The program initializes the PMS5003 sensor and GSM module (if available).
 * - Data is collected at regular intervals and logged in memory or on an SD card.
 * - Data is transmitted to a server at specified intervals.
 * - Failed transmissions are stored and retried later.
 * - The program uses the ArduinoJson library for JSON handling and the TimeLib library for time management.
 * - SD card operations are performed using a custom file handling API.
 *
 * @note The program assumes the presence of specific hardware components, including an SD card module, PMS5003 sensor,
 * and GSM module. It also assumes that the GSM module supports GPRS and can fetch network time.
 *
 * @author Gdieon Maina
 * @date 2025-04-25
 * @version 1.1.0
 *
 * @dependencies
 * - ArduinoJson
 * - TimeLib
 * - PMserial
 * - SD_handler
 * - GSM_handler
 *
 * @hardware
 * - ESP32 microcontroller
 * - PMS5003 sensor
 * - GSM module
 * - SD card module
 *
 * @todo
 * - Implement support for other network connections (e.g., WiFi, LoRa).
 * - Improve JSON validation logic.
 * - Add error handling for SD card operations.
 * - Optimize memory usage for large data sets.
 * - Improve time handling.
 * - Improve power management for battery-operated devices.
 */

#include "global_configs.h"
#include "PMserial.h"
#include "SD_handler.h"
#include "GSM_handler.h"
#include <TimeLib.h>
#include <ArduinoJson.h>
#include <ESP32Time.h>

SerialPM pms(PMS5003, PM_SERIAL_RX, PM_SERIAL_TX); // PMSx003, RX, TX
unsigned long act_milli;
unsigned long last_read_pms = 0;
int sampling_interval = 5 * 60 * 1000; // 5 minutes
unsigned long starttime = 0;
unsigned sending_intervall_ms = 30 * 60 * 1000; // 30 minutes
unsigned long count_sends = 0;
char csv_header[255] = "timestamp,value_type,value,unit,sensor_type";

bool SD_Attached = false;

#define REASSIGN_PINS 1
int SD_SCK = 38;
int SD_MISO = 41;
int SD_MOSI = 40;
int SD_CS = 39;

char ROOT_DIR[24] = {};
char BASE_SENSORS_DATA_DIR[20] = "/SENSORSDATA";
char CURRENT_SENSORS_DATA_DIR[128] = {};
char SENSORS_JSON_DATA_PATH[128] = {};
char SENSORS_CSV_DATA_PATH[128] = {};
char SENSORS_FAILED_DATA_SEND_STORE_FILE[40] = "failed_send_payloads.txt";
char SENSORS_FAILED_DATA_SEND_STORE_PATH[128] = {};

char esp_chipid[18] = {};

bool send_now = false;

ESP32Time RTC;
char time_buff[32] = {};
const char *ISO_time_format = "%Y-%m-%dT%H:%M:%S"; // ISO 8601 format
struct datetimetz
{
    tmElements_t datetime;
    time_t timestamp;
    char timezone[6] = {}; // e.g. +0300 // +03
} esp_datetime_tz;

enum SensorAPN_PIN
{
    PMS = PMS_API_PIN,
    DHT = 7
};

enum DATA_LOGGERS
{
    JSON,
    CSV
};

struct LOGGER
{
    const char *name;
    char *path;
    DATA_LOGGERS type;
    static const int MAX_ENTRIES = 48;
    static const int ENTRY_SIZE = 255;
    char DATA_STORE[MAX_ENTRIES][ENTRY_SIZE] = {};
    int log_count = 0;
} JSON_PAYLOAD_LOGGER, CSV_PAYLOAD_LOGGER;

void getPMSREADINGS();
void printPM_values();
void printPM_Error();
void add_value2JSON_array(JsonArray arr, const char *key, uint16_t &value);
void generateJSON_payload(char *res, JsonDocument &data, const char *timestamp, SensorAPN_PIN pin, size_t size);
bool validateJson(const char *input);
bool sendData(const char *data, const int _pin, const char *host, const char *url);
datetimetz extractDateTime(String datetimeStr);
String formatDateTime(time_t t, String timezone);
String getRTCdatetimetz(const char *format, char *timezone);
void init_memory_loggers();
void init_SD_loggers();
void getMonthName(int month_num, char *month);
void readSendDelete(const char *datafile);
void initCalender(int year, int month);
void updateCalendarFromRTC();
void memoryDataLog(LOGGER &logger, const char *data);
void fileDataLog(LOGGER &logger);
void resetLogger(LOGGER &logger);
void sendFromMemoryLog(LOGGER &logger);

template <typename T>
void generateCSV_payload(char *res, size_t res_size, const char *timestamp, const char *value_type, T value, const char *unit, const char *sensor_type)
{
    // ToDo: check if the value is a string or number (int, float, etc.)
    snprintf(res, res_size, "%s,%s,%d,%s,%s", timestamp, value_type, value, unit, sensor_type);
}

enum Month
{
    _JAN = 1,
    _FEB = 2,
    _MAR = 3,
    _APR = 4,
    _MAY = 5,
    _JUN = 6,
    _JUL = 7,
    _AUG = 8,
    _SEP = 9,
    _OCT = 10,
    _NOV = 11,
    _DEC = 12

};

int current_year, current_month = 0;

void setup()
{
    Serial.begin(115200);

    uint64_t chipid_num;
    chipid_num = ESP.getEfuseMac();
    snprintf(esp_chipid, sizeof(esp_chipid), "%llX", chipid_num);
    Serial.print("ESP32 Chip ID: ");
    Serial.println(esp_chipid);
    Serial.println("Initializing PMS5003 sensor");
    init_memory_loggers();
    pms.init();
    delay(2000);
    pms.sleep();

    if (gsm_capable)
    {
        if (GSM_Serial_begin())
        {

            if (!GSM_init())
            {
                Serial.println("GSM not fully configured");
                Serial.print("Failure point: ");
                Serial.println(GSM_INIT_ERROR);
                Serial.println();
                return;
            }
            else
            {
                GSM_CONNECTED = true;

                while (!register_to_network()) // ! INFINITE LOOP!
                {
                    Serial.println("Retrying network registration...");
                }

                // GPRS init

                if (!GPRS_init())
                {
                    Serial.println("Failed to init GPRS");
                }
                else
                {
                    Serial.println("GPRS initialized!");
                }

                if (getNetworkTime(time_buff))
                {

                    // update RTC time and calendar;
                    Serial.println("GSM Network Time: " + String(time_buff));
                    esp_datetime_tz = extractDateTime(String(time_buff));
                    RTC.setTime(esp_datetime_tz.timestamp);
                    initCalender(RTC.getYear(), RTC.getMonth() + 1);
                }
                else
                {
                    Serial.println("Failed to fetch time from network");
                }
            }

            GSM_sleep();
        }
        else
        {
            Serial.println("Could not communicate to GSM module.");
        }
    }

    else
    {
        // connectWifi();
        Serial.println("Support for other network connections not yet implemented");
    }

// SD INIT
#ifdef REASSIGN_PINS
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (SD_Init(SD_CS))
    {
#else
    if (SD_Init(-1))
    {
#endif

        SD_Attached = SDattached();

        if (SD_Attached)
        {
            init_SD_loggers();
        }
    }

    starttime = millis();
}

void loop()
{
    unsigned sum_send_time = 0;
    act_milli = millis();
    send_now = act_milli - starttime > sending_intervall_ms;
    if (act_milli - last_read_pms > sampling_interval)
    {
        getPMSREADINGS();
    }

    if (send_now)
    {

        if (!GPRS_CONNECTED)
        {
            GPRS_init();
        }
        else
        {

            // Send data from memory loggers
            sendFromMemoryLog(JSON_PAYLOAD_LOGGER);
            // send payloads from the files that stores data that failed posting previously
            readSendDelete(SENSORS_FAILED_DATA_SEND_STORE_PATH);

            // Serial.println("Time for Sending (ms): " + String(sum_send_time));

            // Serial.println("Sent data counts: " + count_sends);
            GSM_sleep();
        }

        starttime = millis();
    }
}

void getPMSREADINGS()
{
    pms.wake();
    delay(30000); // wait for 30 seconds warm-up to get an accurate reading

    // pms.sleep();
    char result_PMS[255] = {};
    pms.read();
    if (pms) // Successfull read
    {
        pms.sleep();
        char read_time[32];
        String datetime = getRTCdatetimetz(ISO_time_format, esp_datetime_tz.timezone);

        last_read_pms = millis();
        // print the results
        printPM_values();

        if (datetime == "") // ! extra validation needed now that RTC is being used
        {
            Serial.println("Datetime is empty...discarding data point");
        }
        else
        {
            updateCalendarFromRTC(); // In case we roll into a new year or month.

            // Generate JSON data
            JsonDocument PM_data_doc;
            JsonArray PM_data = PM_data_doc.to<JsonArray>();

            add_value2JSON_array(PM_data, "P0", pms.pm01);
            add_value2JSON_array(PM_data, "P1", pms.pm25);
            add_value2JSON_array(PM_data, "P2", pms.pm10);

            // serializeJsonPretty(PM_data_doc, Serial);
            generateJSON_payload(result_PMS, PM_data_doc, datetime.c_str(), SensorAPN_PIN::PMS, sizeof(result_PMS));

            memoryDataLog(JSON_PAYLOAD_LOGGER, result_PMS);

            // Generate CSV data and log to memory

            generateCSV_payload(result_PMS, sizeof(result_PMS), datetime.c_str(), "PM0", pms.pm01, "ug/m3", "PMS");
            memoryDataLog(CSV_PAYLOAD_LOGGER, result_PMS);
            generateCSV_payload(result_PMS, sizeof(result_PMS), datetime.c_str(), "PM2.5", pms.pm25, "ug/m3", "PMS");
            memoryDataLog(CSV_PAYLOAD_LOGGER, result_PMS);
            generateCSV_payload(result_PMS, sizeof(result_PMS), datetime.c_str(), "PM10", pms.pm10, "ug/m3", "PMS");
            memoryDataLog(CSV_PAYLOAD_LOGGER, result_PMS);
        }
    }
    else // something went wrong
    {
        pms.sleep();
        printPM_Error();
    }
}

void printPM_values()
{
    Serial.print(F("PM1.0 "));
    Serial.print(pms.pm01);
    Serial.print(F(", "));
    Serial.print(F("PM2.5 "));
    Serial.print(pms.pm25);
    Serial.print(F(", "));
    Serial.print(F("PM10 "));
    Serial.print(pms.pm10);
    Serial.println(F(" [ug/m3]"));

    // if (pms.has_number_concentration())
    // {
    //     Serial.print(F("N0.3 "));
    //     Serial.print(pms.n0p3);
    //     Serial.print(F(", "));
    //     Serial.print(F("N0.5 "));
    //     Serial.print(pms.n0p5);
    //     Serial.print(F(", "));
    //     Serial.print(F("N1.0 "));
    //     Serial.print(pms.n1p0);
    //     Serial.print(F(", "));
    //     Serial.print(F("N2.5 "));
    //     Serial.print(pms.n2p5);
    //     Serial.print(F(", "));
    //     Serial.print(F("N5.0 "));
    //     Serial.print(pms.n5p0);
    //     Serial.print(F(", "));
    //     Serial.print(F("N10 "));
    //     Serial.print(pms.n10p0);
    //     Serial.println(F(" [#/100cc]"));
    // }

    // if (pms.has_temperature_humidity() || pms.has_formaldehyde())
    // {
    //     Serial.print(pms.temp, 1);
    //     Serial.print(F(" °C"));
    //     Serial.print(F(", "));
    //     Serial.print(pms.rhum, 1);
    //     Serial.print(F(" %rh"));
    //     Serial.print(F(", "));
    //     Serial.print(pms.hcho, 2);
    //     Serial.println(F(" mg/m3 HCHO"));
    // }
}

void printPM_Error()
{
    Serial.print(F("Error reading PMS5003 sensor: "));
    switch (pms.status)
    {
    case pms.OK: // should never come here
        break;   // included to compile without warnings
    case pms.ERROR_TIMEOUT:
        Serial.println(F(PMS_ERROR_TIMEOUT));
        break;
    case pms.ERROR_MSG_UNKNOWN:
        Serial.println(F(PMS_ERROR_MSG_UNKNOWN));
        break;
    case pms.ERROR_MSG_HEADER:
        Serial.println(F(PMS_ERROR_MSG_HEADER));
        break;
    case pms.ERROR_MSG_BODY:
        Serial.println(F(PMS_ERROR_MSG_BODY));
        break;
    case pms.ERROR_MSG_START:
        Serial.println(F(PMS_ERROR_MSG_START));
        break;
    case pms.ERROR_MSG_LENGTH:
        Serial.println(F(PMS_ERROR_MSG_LENGTH));
        break;
    case pms.ERROR_MSG_CKSUM:
        Serial.println(F(PMS_ERROR_MSG_CKSUM));
        break;
    case pms.ERROR_PMS_TYPE:
        Serial.println(F(PMS_ERROR_PMS_TYPE));
        break;
    }
}

/**
    @brief Generate JSON payload
    @param res : buffer to store the generated JSON payload
    @param data : JSON document containing the sensor data
    @param timestamp : timestamp of the data
    @param pin : sensor pin type as configured in the API
    @param size : size of the buffer
    @return : void
**/
void generateJSON_payload(char *res, JsonDocument &data, const char *timestamp, SensorAPN_PIN pin, size_t size)
{
    JsonDocument payload;

    payload["software_version"] = "NRZ-2020-129";
    payload["timestamp"] = timestamp;
    payload["sensordatavalues"] = data;

    // char sensor_type[24] = ",\"type\":\"";
    switch (pin)
    {
    case SensorAPN_PIN::PMS:
        // strcat(res, "PMS\"");
        payload["sensor_type"] = "PMS";
        payload["APN_PIN"] = pin;
        break;
    case SensorAPN_PIN::DHT:
        // strcat(res, "DHT\"");
        payload["sensor_type"] = "DHT";
        payload["APN_PIN"] = pin;
        break;
    default:
        Serial.println("Unsupported sensor pin");
        break;
    }

    serializeJson(payload, res, size);
}

datetimetz extractDateTime(String datetimeStr)
{
    datetimetz dtz;
    dtz.datetime = {0, 0, 0, 0, 0, 0, 0};
    dtz.timestamp = 0;

    Serial.println("Received date string: " + datetimeStr); //! format looks like "25/02/24,05:55:53+00" and may include the quotes!

    // check if received string is empty
    if (datetimeStr == "")
    {
        Serial.println("Datetime string is empty");

        return dtz;
    }

    // check if the datetime string has leading or trailing quotes
    if (datetimeStr[0] == '"', datetimeStr[datetimeStr.length() - 1] == '"')
    {
        // remove the first and last character of the string (")
        datetimeStr = datetimeStr.substring(1, datetimeStr.length() - 1);
    }

    // Parse the datetime string

    int _year = datetimeStr.substring(0, 2).toInt();
    int _month = datetimeStr.substring(3, 5).toInt();
    int _day = datetimeStr.substring(6, 8).toInt();
    int _hour = datetimeStr.substring(9, 11).toInt();
    int _minute = datetimeStr.substring(12, 14).toInt();
    int _second = datetimeStr.substring(15, 17).toInt();

    // perform sanity check on the parsed values
    if (_year < 0 || _year > 99 || _month < 1 || _month > 12 || _day < 1 || _day > 31 ||
        _hour < 0 || _hour > 23 || _minute < 0 || _minute > 59 || _second < 0 || _second > 59)
    {
        Serial.println("Invalid date/time values");
        return dtz;
    }

#if defined(QUECTEL)

    // time zone = indicates the difference, expressed in quarters of an hour, between the local time and GMT; range: -48 to +56)
    int tz = datetimeStr.substring(18).toInt() / 4;
    String timezone = datetimeStr.substring(17, 18); // extract timezone sign
    if (tz < 10)
    {
        timezone += "0" + String(tz);
    }
    else
    {
        timezone += String(tz);
    }
#else
    String timezone = datetimeStr.substring(17); // +00

#endif
    strncpy(dtz.timezone, timezone.c_str(), 6); // copy timezone to the provided buffer

    Serial.println("Day: " + String(_day));
    Serial.println("Month: " + String(_month));
    Serial.println("Year: " + String(_year));
    Serial.println("Hour: " + String(_hour));
    Serial.println("Minute: " + String(_minute));
    Serial.println("Second: " + String(_second));

    // Adjust year for TimeLib (TimeLib expects years since 1970)
    _year += 2000; // Assuming 24 is 2024
    _year -= 1970;

    dtz.datetime.Second = _second;
    dtz.datetime.Minute = _minute;
    dtz.datetime.Hour = _hour;
    dtz.datetime.Day = _day;
    dtz.datetime.Month = _month;
    dtz.datetime.Year = _year;

    // Create a time_t value
    dtz.timestamp = makeTime(dtz.datetime);
    Serial.print("Parsed timestamp: ");
    Serial.println(dtz.timestamp); // Print the time_t value

    return dtz;
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

String getRTCdatetimetz(const char *format, char *timezone)
{
    String datetimetz = RTC.getTime(format);
    datetimetz += timezone;
    return datetimetz;
}

/*****************************************************************
 * send data to rest api                                         *
 *****************************************************************/
/**
    @brief: Send data to the server
    @param data : JSON payload to send
    @param _pin : pin number of the sensor as configured in the API
    @param host : host name of the server
    @param url : url path to send the data
    @return: true if data is sent successfully, false otherwise
**/
bool sendData(const char *data, const int _pin, const char *host, const char *url)
{
    // unsigned long start_send = millis();

    char gprs_url[64] = {};
    strcat(gprs_url, host);
    strcat(gprs_url, url);

    char pin[4];
    itoa(_pin, pin, 10);

    if (gsm_capable && GPRS_CONNECTED)
    {

        int retry_count = 0;
        uint8_t statuscode = 0;
        int16_t length;

#ifdef QUECTEL
        char Quectel_headers[3][40] = {};
        strcat(Quectel_headers[0], "X-PIN: ");
        strcat(Quectel_headers[0], pin);

        strcat(Quectel_headers[1], "X-Sensor: ");
        strcat(Quectel_headers[1], SENSOR_PREFIX);
        strcat(Quectel_headers[1], esp_chipid);
        strcat(Quectel_headers[2], "Content-Type: application/json");

        // int header_size = sizeof(Quectel_headers) / sizeof(Quectel_headers[0]);

        QUECTEL_POST(gprs_url, Quectel_headers, 3, data, strlen(data), statuscode);

        if (!(statuscode == 200 || statuscode == 201))
        {
            return false;
        }

        // ToDo: close HTTP session/ PDP context
#endif
    }

    // #if defined(ESP8266)
    //     wdt_reset();
    // #endif
    //     yield();
    //     return millis() - start_send;

    return true;
}

void init_memory_loggers()
{
    // Initialize memory loggers
    JSON_PAYLOAD_LOGGER.name = "JSON";
    JSON_PAYLOAD_LOGGER.path = SENSORS_JSON_DATA_PATH;
    JSON_PAYLOAD_LOGGER.type = DATA_LOGGERS::JSON;

    CSV_PAYLOAD_LOGGER.name = "CSV";
    CSV_PAYLOAD_LOGGER.path = SENSORS_CSV_DATA_PATH;
    CSV_PAYLOAD_LOGGER.type = DATA_LOGGERS::CSV;
}

/// @brief Init directories for logging files
void init_SD_loggers()
{
    // Root directory
    strcpy(ROOT_DIR, "/");
    strcat(ROOT_DIR, SENSOR_PREFIX);
    strcat(ROOT_DIR, esp_chipid);
    createDir(SD, ROOT_DIR);

    if (current_year != 0 && current_month != 0)
    {
        char _year[4] = {};
        strcpy(CURRENT_SENSORS_DATA_DIR, ROOT_DIR);
        strcat(CURRENT_SENSORS_DATA_DIR, BASE_SENSORS_DATA_DIR);

        createDir(SD, CURRENT_SENSORS_DATA_DIR); // Create base sensor directory "/ESP_CHIP_ID/SENSORSDATA"

        itoa(current_year, _year, 10);
        strcat(CURRENT_SENSORS_DATA_DIR, "/");
        strcat(CURRENT_SENSORS_DATA_DIR, _year);

        createDir(SD, CURRENT_SENSORS_DATA_DIR); // Create year directory "/ESP_CHIP_ID/SENSORSDATA/2025"
    }

    else
    {
        Serial.println("Year or month not set");
        return;
    }

    memset(SENSORS_JSON_DATA_PATH, 0, sizeof(SENSORS_JSON_DATA_PATH));
    char month[3] = {};
    getMonthName(current_month, month);

    // Update sensors JSON data path
    strcpy(SENSORS_JSON_DATA_PATH, CURRENT_SENSORS_DATA_DIR);
    strcat(SENSORS_JSON_DATA_PATH, "/");
    strcat(SENSORS_JSON_DATA_PATH, month);
    strcat(SENSORS_JSON_DATA_PATH, ".txt");

    // Update sensors CSV data path
    strcpy(SENSORS_CSV_DATA_PATH, CURRENT_SENSORS_DATA_DIR);
    strcat(SENSORS_CSV_DATA_PATH, "/");
    strcat(SENSORS_CSV_DATA_PATH, month);
    strcat(SENSORS_CSV_DATA_PATH, ".csv");

    int _from = 0;
    int _to = 0;
    if (readLine(SD, SENSORS_CSV_DATA_PATH, _to, _from, true) != (String)csv_header)
    {
        appendFile(SD, SENSORS_CSV_DATA_PATH, csv_header);
    }

    // Init logger paths
    strcpy(SENSORS_FAILED_DATA_SEND_STORE_PATH, ROOT_DIR);
    strcat(SENSORS_FAILED_DATA_SEND_STORE_PATH, BASE_SENSORS_DATA_DIR);
    strcat(SENSORS_FAILED_DATA_SEND_STORE_PATH, "/");
    strcat(SENSORS_FAILED_DATA_SEND_STORE_PATH, SENSORS_FAILED_DATA_SEND_STORE_FILE);
    strcat(SENSORS_FAILED_DATA_SEND_STORE_PATH, "\0");

    // write files to SD
    // writeFile(SD, SENSORS_JSON_DATA_PATH, "");                   // create file if it does not exist
    // writeFile(SD, SENSORS_FAILED_DATA_SEND_STORE_PATH, ""); // create file if it does not exist

    // Debug runtime logger;
}

/**
    @brief convert number of month to name
    @param month_num : range from 1 to 12
    @param month : name of the month
    @return : void
**/
void getMonthName(int month_num, char *month)
{
    switch (month_num)
    {
    case (Month::_JAN):
        strcpy(month, "JAN");
        break;

    case (Month::_FEB):
        strcpy(month, "FEB");
        break;

    case (Month::_MAR):
        strcpy(month, "MAR");
        break;

    case (Month::_APR):
        strcpy(month, "APR");
        break;

    case (Month::_MAY):
        strcpy(month, "MAY");
        break;

    case (Month::_JUN):
        strcpy(month, "JUN");
        break;

    case (Month::_JUL):
        strcpy(month, "JUL");
        break;
    case (Month::_AUG):
        strcpy(month, "AUG");
        break;
    case (Month::_SEP):
        strcpy(month, "SEP");
        break;
    case (Month::_OCT):
        strcpy(month, "OCT");
        break;
    case (Month::_NOV):
        strcpy(month, "NOV");
        break;
    case (Month::_DEC):
        strcpy(month, "DEC");
        break;
    }
}

/// @brief : Read data from SD card and send it to the server
/// @param datafile : file name to read from
/// @return : void
/// @note : The function will read the data from the file and send it to the server. If the send fails, the data will be appended to a temporary file for later sending.
/// @note : The function will also update the file contents to remove the data that was sent successfully.
void readSendDelete(const char *datafile)
{
    String data;
    const char *tempFile = "/temp_sensor_payload.txt";

    Serial.println("Attempting to send data that previoudly failed to send.");

    int next_byte = -1;
    int next_line_index = 0;

    // readline continously
    do
    {

        data = readLine(SD, datafile, next_byte, next_line_index, false);

        if (next_byte == -1) // End of file reached
        {
            Serial.println("End of file read");
        }

        if (!validateJson(data.c_str()))
        {
            Serial.println("Invalid JSON data: " + data);
            continue;
        }

        if (data != "")
        {
            // Todo: check senor type to determine which API pin to use
            // Attempt send payload
            // Serial.println("Attempting to send data from SD card: " + data);
            if (!sendData(data.c_str(), PMS_API_PIN, HOST_CFA, URL_CFA))
            {
                // store data in temp file
                appendFile(SD, tempFile, data.c_str(), true); //? does FS lib support opening multiple files? Otherwise close the previously opened file.
            }
        }
    } while (next_byte != -1);

    updateFileContents(SD, datafile, tempFile);

    // close files
    closeFile(SD, datafile);
}

void updateCalendarFromRTC()
{
    bool calendarUpdated = false;
    int year = RTC.getYear();
    int month = RTC.getMonth() + 1; // ESP32Time month is 0 based

    if (year > current_year)
    {
        Serial.print("Updating year from: ");
        Serial.print(current_year);
        Serial.print(" to: ");
        Serial.println(year);

        current_year = year;
        current_month = month; // Also update the month. Ideally, Jan.
        calendarUpdated = true;
    }
    else if (month > current_month)
    {
        Serial.print("Updating year from: ");
        Serial.print(current_month);
        Serial.print(" to: ");
        Serial.println(month);

        current_month = month;
        calendarUpdated = true;
    }
    if (calendarUpdated)
    {
        init_SD_loggers();
    }
}

void initCalender(int year, int month)
{
    Serial.print("Initializing calendar with year: ");
    Serial.print(year);
    Serial.print(" and month: ");
    Serial.println(month);
    if (year != 0 && month != 0)
    {
        current_year = year;
        current_month = month;
    }
}

/**
    @brief Add value to JSON array
    @param arr : JSON array to add the value to
    @param key : key for the value
    @param value : value to add
    @return : void
**/
void add_value2JSON_array(JsonArray arr, const char *key, uint16_t &value)
{
    JsonDocument doc;
    doc["value_type"] = key;
    doc["value"] = value;
    arr.add(doc);
}

/**
    @brief Validate JSON data
    @param input : JSON data to validate
    @return : true if valid, false otherwise
    @note : //! This function is not full proof. Raw strings that don't look like incomplete or empty JSON are validated.
**/
bool validateJson(const char *input)
{
    JsonDocument doc, filter;
    return deserializeJson(doc, input, DeserializationOption::Filter(filter)) == DeserializationError::Ok;
}

/**
    @brief Log data to memory
    @param logger : logger to log the data to
    @param data : data to log
    @return : void
    @note : The function will log the data to the logger. If the logger is full, it will append the data to a file.
**/
void memoryDataLog(LOGGER &logger, const char *data)
{

    if (logger.log_count < logger.MAX_ENTRIES)
    {
        strcpy(logger.DATA_STORE[logger.log_count], data);
        Serial.println("Logged data: " + String(logger.DATA_STORE[logger.log_count]));
        logger.log_count++;
    }
    else
    {
        Serial.println("Logger data log count exceeded for " + String(logger.name));
        switch (logger.type)
        {
        case DATA_LOGGERS::JSON:
            // Append to JSON file
            fileDataLog(logger);
            send_now = true;
            break;
        case DATA_LOGGERS::CSV:
            fileDataLog(logger);
            resetLogger(logger);
            break;
        }
    }
}

/**
    @brief Log data to file
    @param logger : logger to log the data to
    @return : void
    @note : The function will log the data to the file. If the file is full, it will append the data to a new file.
**/
void fileDataLog(LOGGER &logger)
{
    Serial.println("Logging data to file: " + String(logger.path));
    for (int i = 0; i < logger.MAX_ENTRIES; i++)
    {
        if (strlen(logger.DATA_STORE[i]) != 0)
        {
            appendFile(SD, logger.path, logger.DATA_STORE[i]);
        }
    }
}

/**
    @brief Reset logger
    @param logger : logger to reset
    @return : void
    @note : The function will reset the logger. It will clear the data store and set the log count to 0.
**/
void resetLogger(LOGGER &logger)
{
    logger.log_count = 0;
    memset(logger.DATA_STORE, 0, sizeof(logger.DATA_STORE)); // ToDo: check if this works
    //? or this logger.DATA_STORE = {};
}

/**
    @brief Send data from memory loggers
    @param logger : logger to send data from
    @return : void
    @note : The function will send the data from the logger. If the send fails, the data will be appended to a file for later sending.
**/
void sendFromMemoryLog(LOGGER &logger)
{
    for (int i = 0; i < logger.log_count; i++)
    {
        if (strlen(logger.DATA_STORE[i]) != 0)
        {
            if (!sendData(logger.DATA_STORE[i], PMS_API_PIN, HOST_CFA, URL_CFA))
            {
                // Append to file for sending later
                appendFile(SD, SENSORS_FAILED_DATA_SEND_STORE_PATH, logger.DATA_STORE[i]);
            }
            memset(logger.DATA_STORE[i], '\0', 255);
        }
    }
    logger.log_count = 0;

    //? call resetLogger(logger) to reset the logger
    //? or just clear the memory
}
