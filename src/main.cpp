#include "global_configs.h"
#include "PMserial.h"
#include "SD_handler.h"
#include "GSM_handler.h"
#include <TimeLib.h>
#include <ArduinoJson.h>

SerialPM pms(PMS5003, PM_SERIAL_RX, PM_SERIAL_TX); // PMSx003, RX, TX
unsigned long act_milli;
unsigned long last_read_pms = 0;
int sampling_interval = 30000;
unsigned long starttime = 0;
unsigned sending_intervall_ms = 1 * 60 * 1000; // 145000;
unsigned long count_sends = 0;

uint8_t sensor_data_log_count = 0;
const int MAX_PAYLOADS = 48;
char sensor_data[MAX_PAYLOADS][255];

bool SD_MOUNT = false;
bool SD_Attached = false;

#define REASSIGN_PINS 1
int SD_SCK = 38;
int SD_MISO = 41;
int SD_MOSI = 40;
int SD_CS = 39;

char ROOT_DIR[24] = {};
char BASE_SENSORS_DATA_DIR[20] = "/SENSORSDATA";
char CURRENT_SENSORS_DATA_DIR[128] = {};
char SENSORS_DATA_PATH[128] = {};
char SENSORS_FAILED_DATA_SEND_STORE_FILE[40] = "failed_send_payloads.txt";
char SENSORS_FAILED_DATA_SEND_STORE_PATH[128] = {};

char esp_chipid[18] = {};

bool send_now = false;

char time_buff[32] = {};

enum SensorAPN_PIN
{
    PMS = PMS_API_PIN,
    DHT = 7
};

void getPMSREADINGS();
void printPM_values();
void printPM_Error();
void add_value2JSON_array(JsonArray arr, const char *key, uint16_t &value);
void generateJSON_payload(char *res, JsonDocument &data, const char *timestamp, SensorAPN_PIN pin, size_t size);
bool sendData(const char *data, const int _pin, const char *host, const char *url);
String extractDateTime(String datetimeStr);
String formatDateTime(time_t t, String timezone);
void init_SD_loggers();
void getMonthName(int month_num, char *month);
void readSendDelete(const char *datafile);
void initCalenderFromNetworkTime();
void updateCalendarFromNetworkTime();

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
int network_year, network_month = 0;

void setup()
{
    Serial.begin(115200);

    uint64_t chipid_num;
    chipid_num = ESP.getEfuseMac();
    snprintf(esp_chipid, sizeof(esp_chipid), "%llX", chipid_num);
    Serial.print("ESP32 Chip ID: ");
    Serial.println(esp_chipid);
    Serial.println("Initializing PMS5003 sensor");
    pms.init();

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
                    Serial.println("Time: " + String(time_buff));
                    extractDateTime(String(time_buff));
                    initCalenderFromNetworkTime();
                }
                else
                {
                    Serial.println("Failed to fetch time from network");
                }
            }
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
    Serial.print("Init SD ......");
    delay(5000);
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS))
    {
#else
    if (!SD.begin())
    {
#endif
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
        init_SD_loggers();
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

        // save data to SD
        for (int i = 0; i < sensor_data_log_count; i++)
        {
            if (strlen(sensor_data[i]) != 0)
            {
                appendFile(SD, SENSORS_DATA_PATH, sensor_data[i]);
            }
        }

        if (!GPRS_CONNECTED)
        {
            GPRS_init();
        }
        else
        {

            // Read data from array and send to server
            for (int i = 0; i < sensor_data_log_count; i++)
            {

                if (strlen(sensor_data[i]) != 0)
                {
                    if (!sendData(sensor_data[i], PMS_API_PIN, HOST_CFA, URL_CFA))
                    {
                        // Append to file for sending later
                        appendFile(SD, SENSORS_FAILED_DATA_SEND_STORE_PATH, sensor_data[i]);
                    }
                    memset(sensor_data[i], '\0', 255);
                }
            }

            // send payloads from the files that stores data that failed posting previously
            readSendDelete(SENSORS_FAILED_DATA_SEND_STORE_PATH);

            // Serial.println("Time for Sending (ms): " + String(sum_send_time));

            // Resetting for next sampling

            sensor_data_log_count = 0;

            count_sends++;
            Serial.println("Sent data counts: " + count_sends);
        }

        starttime = millis();
    }
}

void getPMSREADINGS()
{
    // pms.wake_up();
    // delay(30000); // wait for 30 seconds to get a reading
    // pms.request_reading();
    // delay(10000); // wait for 10 seconds to get a reading
    // pms.sleep();
    char result_PMS[255] = {};
    pms.read();
    if (pms) // Successfull read
    {

        char read_time[32];
        String datetime = "";

        last_read_pms = millis();
        // print the results
        printPM_values();

        if (getNetworkTime(read_time))
        {
            datetime = extractDateTime(String(read_time));
        }

        if (datetime == "")
        {
            Serial.println("Datetime is empty...discarding data point");
        }
        else
        {
            updateCalendarFromNetworkTime(); // In case we roll into a new year or month.

            if (sensor_data_log_count < MAX_PAYLOADS)
            {
                // Add values to JSON
                JsonDocument PM_data_doc;
                JsonArray PM_data = PM_data_doc.to<JsonArray>();

                add_value2JSON_array(PM_data, "P0", pms.pm01);
                add_value2JSON_array(PM_data, "P1", pms.pm25);
                add_value2JSON_array(PM_data, "P2", pms.pm10);

                // serializeJsonPretty(PM_data_doc, Serial);
                generateJSON_payload(result_PMS, PM_data_doc, datetime.c_str(), SensorAPN_PIN::PMS, sizeof(result_PMS));

                Serial.print("\nresult_PMS JSON: ");
                Serial.println(result_PMS);

                // Store in payload sensor data buffer

                strcat(sensor_data[sensor_data_log_count], result_PMS);
                Serial.print("Sensor data: ");
                Serial.println(sensor_data[sensor_data_log_count]);
                sensor_data_log_count++;
            }
            else
            {
                Serial.println("Sensor data log count exceeded");
            }
        }
    }
    else // something went wrong
    {
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

String extractDateTime(String datetimeStr)
{
    Serial.println("Received date string: " + datetimeStr); //! format looks like "25/02/24,05:55:53+00" and may include the quotes!

    // check if received string is empty
    if (datetimeStr == "")
    {
        Serial.println("Datetime string is empty");
        return "";
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
        return "";
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

    // Serial.println("Day: " + String(day));
    // Serial.println("Month: " + String(month));
    // Serial.println("Year: " + String(year));
    // Serial.println("Hour: " + String(hour));
    // Serial.println("Minute: " + String(minute));
    // Serial.println("Second: " + String(second));

    // Adjust year for TimeLib (TimeLib expects years since 1970)
    _year += 2000; // Assuming 24 is 2024
    _year -= 1970;

    // Create a tmElements_t struct
    tmElements_t tm;
    tm.Second = _second;
    tm.Minute = _minute;
    tm.Hour = _hour;
    tm.Day = _day;
    tm.Month = _month;
    tm.Year = _year;

    // Create a time_t value
    time_t t = makeTime(tm);

    network_year = year(t);
    network_month = month(t);

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

/*****************************************************************
 * send data to rest api                                         *
 *****************************************************************/
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

    memset(SENSORS_DATA_PATH, 0, sizeof(SENSORS_DATA_PATH));
    char month[3] = {};
    getMonthName(current_month, month);

    // Update sensors data path
    strcpy(SENSORS_DATA_PATH, CURRENT_SENSORS_DATA_DIR);
    strcat(SENSORS_DATA_PATH, "/");
    strcat(SENSORS_DATA_PATH, month);
    strcat(SENSORS_DATA_PATH, ".txt");

    // Init logger paths
    strcpy(SENSORS_FAILED_DATA_SEND_STORE_PATH, ROOT_DIR);
    strcat(SENSORS_FAILED_DATA_SEND_STORE_PATH, BASE_SENSORS_DATA_DIR);
    strcat(SENSORS_FAILED_DATA_SEND_STORE_PATH, "/");
    strcat(SENSORS_FAILED_DATA_SEND_STORE_PATH, SENSORS_FAILED_DATA_SEND_STORE_FILE);
    strcat(SENSORS_FAILED_DATA_SEND_STORE_PATH, "\0");

    // write files to SD
    // writeFile(SD, SENSORS_DATA_PATH, "");                   // create file if it does not exist
    // writeFile(SD, SENSORS_FAILED_DATA_SEND_STORE_PATH, ""); // create file if it does not exist

    // Debug runtime logger;
}

/// @brief convert number of month to name
/// @param month_num : range from 1 to 12
/// @param month : name of the month
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

void readSendDelete(const char *datafile)
{
    bool EndOfFile = false;
    String data;
    const char *tempFile = "/temp_sensor_payload.txt";

    Serial.println("Attempting to send data that previoudly failed to send.");

    while (!EndOfFile)
    {

        data = readLine(SD, datafile, EndOfFile, false);
        // readline continously
        if (data != "")
        {

            // check type of payload //! for now we assume it's only PM data
            // Todo: validate JSON
            // Todo: check senor type to determine which API pin to use
            // Attempt send payload
            if (!sendData(data.c_str(), PMS_API_PIN, HOST_CFA, URL_CFA))
            {
                // store data in temp file
                appendFile(SD, tempFile, data.c_str(), false); //? does FS lib support opening multiple files? Otherwise close the previously opened file.
            }
        }
    }

    updateFileContents(SD, datafile, tempFile);

    // close files
    closeFile(SD, datafile);
}

void updateCalendarFromNetworkTime()
{
    bool calendarUpdated = false;

    if (network_year > current_year)
    {
        Serial.print("Updating year from: ");
        Serial.print(current_year);
        Serial.print(" to: ");
        Serial.println(network_year);

        current_year = network_year;
        current_month = network_month; // Also update the month. Ideally, Jan.
        calendarUpdated = true;
    }
    else if (network_month > current_month)
    {
        Serial.print("Updating year from: ");
        Serial.print(current_month);
        Serial.print(" to: ");
        Serial.println(network_month);

        current_month = network_month;
        calendarUpdated = true;
    }
    if (calendarUpdated)
    {
        init_SD_loggers();
    }
}

void initCalenderFromNetworkTime()
{
    if (network_year != 0 && network_month != 0)
    {
        current_year = network_year;
        current_month = network_month;
    }
}

void add_value2JSON_array(JsonArray arr, const char *key, uint16_t &value)
{
    JsonDocument doc;
    doc["value_type"] = key;
    doc["value"] = value;
    arr.add(doc);
}