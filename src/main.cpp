#include "global_configs.h"
#include "PMserial.h"
#include "SD_handler.h"
#include "GSM_handler.h"
#include <TimeLib.h>

SerialPM pms(PMS5003, PM_SERIAL_RX, PM_SERIAL_TX); // PMSx003, RX, TX
unsigned long act_milli;
unsigned long last_read_pms = 0;
int sampling_interval = 30000;
unsigned long starttime = 0;
unsigned sending_intervall_ms = 1 * 60 * 1000; // 145000;
unsigned long count_sends = 0;

uint8_t sensor_data_log_count = 0;
const int MAX_STRINGS = 48;
char sensor_data[MAX_STRINGS][255];

bool SD_MOUNT = false;
bool SD_Attached = false;

#define REASSIGN_PINS 1
int SD_SCK = 38;
int SD_MISO = 41;
int SD_MOSI = 40;
int SD_CS = 39;

char SENSORS_DATA_DIR[20] = "/SENSORSDATA";
char SENSORS_DATA_PATH[40] = "/SENSORSDATA/sensordatalog.txt";

char esp_chipid[18] = {};

bool send_now = false;

char time_buff[32] = {};

void printPM_values();
void printPM_Error();
static void add_Value2Json(char *res, char *value_type, uint16_t &value);
void generateJSON_payload(char *res, char *data, const char *timestamp);
static unsigned long sendData(const char *data, const int _pin, const char *host, const char *url);
String extractDateTime(String datetimeStr);
String formatDateTime(time_t t, String timezone);

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
        createDir(SD, SENSORS_DATA_DIR);
    }

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

    starttime = millis();
}

void loop()
{
    static char result_PMS[255] = {};
    unsigned sum_send_time = 0;
    act_milli = millis();
    send_now = act_milli - starttime > sending_intervall_ms;
    if (act_milli - last_read_pms > sampling_interval)
    {
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
                if (sensor_data_log_count < MAX_STRINGS)
                {
                    // Add values to JSON
                    memset(result_PMS, 0, 255);

                    char PM_data[255] = {};
                    // add_Value2Json(PM_data, "PM1", pms.pm01);
                    // add_Value2Json(PM_data, "PM2", pms.pm25);
                    // add_Value2Json(PM_data, "PM10", pms.pm10);
                    add_Value2Json(PM_data, "P0", pms.pm01);
                    add_Value2Json(PM_data, "P1", pms.pm25);
                    add_Value2Json(PM_data, "P2", pms.pm10);

                    generateJSON_payload(result_PMS, PM_data, datetime.c_str());

                    Serial.print("result_PMS JSON: ");
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
                    sum_send_time += sendData(sensor_data[i], PMS_API_PIN, HOST_CFA, URL_CFA);
                    memset(sensor_data[i], '\0', 255);
                }
            }

            Serial.println("Time for Sending (ms): " + String(sum_send_time));

            // Resetting for next sampling
            sum_send_time = 0;
            sensor_data_log_count = 0;

            count_sends++;
            Serial.println("Sent data counts: " + count_sends);
        }

        starttime = millis();
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

void add_Value2Json(char *res, char *value_type, uint16_t &value)
{
    char value_str[64] = {}; // ! make sure this is big enough
    char char_value[18];
    // memset(value_str, 0, 64);
    itoa(value, char_value, 10);

    // Serial.print("Checking if value_str has something: ");
    // Serial.println(value_str);
    // Serial.println(value_str[0], HEX);
    // Serial.println(value_str[1], HEX);

    strcat(value_str, "{\"value_type\":\"");
    strcat(value_str, value_type); // add value type
    strcat(value_str, "\",\"value\":\"");
    strcat(value_str, char_value); // add value
    strcat(value_str, "\"},");     // last part with trailing comma

    strcat(res, value_str);

    // Serial.print("JSON value: ");
    // Serial.println(value_str);
}

void generateJSON_payload(char *res, char *data, const char *timestamp)
{

    strcpy(res, "{\"software_version\": \"NRZ-2020-129\",");
    strcat(res, "\"timestamp\": \"");
    strcat(res, timestamp);
    strcat(res, "\",");
    strcat(res, "\"sensordatavalues\":[");
    strcat(res, data);
    char *trailing_comma = strrchr(res, ',');
    if (trailing_comma)
    {
        res[trailing_comma - res] = '\0';
    }
    strcat(res, "]}");
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

    int year = datetimeStr.substring(0, 2).toInt();
    int month = datetimeStr.substring(3, 5).toInt();
    int day = datetimeStr.substring(6, 8).toInt();
    int hour = datetimeStr.substring(9, 11).toInt();
    int minute = datetimeStr.substring(12, 14).toInt();
    int second = datetimeStr.substring(15, 17).toInt();

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

/*****************************************************************
 * send data to rest api                                         *
 *****************************************************************/
static unsigned long sendData(const char *data, const int _pin, const char *host, const char *url)
{

    unsigned long start_send = millis();

    char gprs_url[64] = {};
    strcat(gprs_url, host);
    strcat(gprs_url, url);

    char pin[4];
    itoa(_pin, pin, 10);

    if (gsm_capable && GPRS_CONNECTED)
    {

        int retry_count = 0;
        uint16_t statuscode;
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

        QUECTEL_POST(gprs_url, Quectel_headers, 3, data, strlen(data));

        // ToDo: close HTTP session/ PDP context
#endif
    }

#if defined(ESP8266)
    wdt_reset();
#endif
    yield();
    return millis() - start_send;
}
