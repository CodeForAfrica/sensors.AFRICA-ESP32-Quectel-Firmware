// HardwareSerial fonaSS(1);
// Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

#include "Adafruit_FONA.h"
#include "global_configs.h"

HardwareSerial fonaSS(2);
HardwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

char SIM_PIN[5] = GSM_PIN;
bool GSM_CONNECTED = false;
bool SIM_AVAILABLE = false;
bool GPRS_CONNECTED = false;
bool SIM_PIN_SET = false;
bool SIM_USABLE = false;
uint16_t CGATT_status;
char SIM_CCID[21] = "";
String GSM_INIT_ERROR = "";
String NETWORK_NAME = "";

// FAIL FLAGS
#ifdef QUECTEL
int HTTPCFG_CONNECT_FAIL = 0;
#endif
int GPRS_INIT_FAIL_COUNT = 0;
int HTTP_POST_FAIL = 0;
int REGISTER_TO_NETWORK_FAIL = 0;

uint16_t HTTPOST_RESPONSE_STATUS;

/**** Function Declacrations **/
bool GSM_init(HardwareSerial *gsm_serial);
bool register_to_network();
static void unlock_pin(char *PIN);
void SIM_PIN_Setup();
bool is_SIMCID_valid();
bool GPRS_init();
void GSM_soft_reset();
void restart_GSM();
void enableGPRS();
bool activateGPRS();
bool deactivateGPRS();
int8_t GPRS_status();
void flushSerial();
void SerialFlush();
void QUECTEL_POST(char *url, String headers[], int header_size, const String &data, int data_length);
bool extractText(char *input, const char *target, char *output, uint8_t output_size, char _until); // ? should go to utils
void get_raw_response(const char *cmd, char *res_buff, size_t buff_size, bool fill_buffer = false, unsigned long timeout = 3000);
int16_t getNumber(char *AT_cmd, char *expected_reply, uint8_t index_from, uint8_t length);
void get_http_response_status(String data, char *HTTP_RESPONSE_STATUS);
bool sendAndCheck(const char *AT_cmd, const char *expected_reply, unsigned long timeout = 1000);

void troubleshoot_GSM();

// Set a decent delay before this to warm up the GSM module
bool GSM_init(HardwareSerial *gsm_serial)
{ // Pass a ptr to SoftwareSerial GSM instance

    pinMode(QUECTEL_PWR_KEY, OUTPUT);
    digitalWrite(QUECTEL_PWR_KEY, HIGH);
    // delay(5000);

    gsm_serial->begin(115200, SERIAL_8N1, MCU_RXD, MCU_TXD);
    String error_msg = "";

    // Check if there is serial communication with a GSM module
    if (!fona.begin(*gsm_serial, fona.LOW_HIGH_LOW, 120))
    {
        error_msg = "Could not find GSM module";
        GSM_INIT_ERROR = error_msg;
        Serial.println(error_msg);
        GSM_CONNECTED = false;
        return false;
    }

    Serial.println("GSM module found!");

    // Check if SIM is inserted
    if (!is_SIMCID_valid())
    {
        error_msg = "Could not get SIM CID";
        GSM_INIT_ERROR = error_msg;
        Serial.println(error_msg);
        return false;
    }

    // Serial.println("Setting up SIM..");

    // SIM_PIN_Setup();

    // if (!SIM_PIN_SET)
    // {
    //     error_msg = "Unable to set SIM PIN";
    //     GSM_INIT_ERROR = error_msg;
    //     Serial.println(error_msg);

    //     return false;
    // }
    // Set if SIM is usable flag
    SIM_USABLE = true;

    // fona.sendCheckReply(F("AT+CMEE=2"), F("OK"));
    sendAndCheck("AT+CMEE=2", "OK");

    return true;
}

bool register_to_network()
{

    String error_msg = "";
    bool registered_to_network = false;
    int retry_count = 0;
    while (!registered_to_network && retry_count < 20)
    {
        int8_t status = getNumber("AT+CREG?\0", "+CREG: ", 2, 1);

        if (status == 1 || status == 5)
        {
            registered_to_network = true;
            break;
        }

        else
        {
            Serial.println("Not registered to network ");
        }

        retry_count++;
        delay(3000);
    }

    if (!registered_to_network)
    {
        error_msg = "Network not registered";
        GSM_INIT_ERROR = error_msg;
        Serial.println(error_msg);
        REGISTER_TO_NETWORK_FAIL += 1;

        // Attempt to enable network registration

        if (!sendAndCheck("AT+CREG=1\0", "OK"))
        {
            Serial.println("Manual network registration failed.");
        }

        if (REGISTER_TO_NETWORK_FAIL > 5)
        {
            GSM_soft_reset();
            //? Check if the SIM card is still there?
            REGISTER_TO_NETWORK_FAIL = 0;
        }
        return false;
    }

    // fona.sendCheckReply(F("AT+COPS?"), F("OK"));
    sendAndCheck("AT+COPS?", "OK");

    return true;
}

static void unlock_pin(char *PIN)
{
    // flushSerial();

    // Attempt to SET PIN if not empty
    Serial.print("GSM CONFIG SET PIN: ");
    Serial.println(PIN);
    Serial.print("Length of PIN");
    Serial.println(strlen(PIN));
    if (strlen(PIN) > 1)
    {
        // debug_outln(F("\nAttempting to Unlock SIM please wait: "), DEBUG_MIN_INFO);
        Serial.print("Attempting to unlock SIM using PIN: ");
        Serial.println(PIN);
        if (!fona.unlockSIM(PIN))
        {
            // debug_outln(F("Failed to Unlock SIM card with pin: "), DEBUG_MIN_INFO);
            Serial.print("Failed to Unlock SIM card with PIN: ");
            // debug_outln(gsm_pin, DEBUG_MIN_INFO);
            Serial.println(PIN);
            SIM_PIN_SET = false;
            return;
        }

        SIM_PIN_SET = true;
    }
}

void SIM_PIN_Setup()
{

    if (sendAndCheck("AT+CPIN?", "+CPIN: READY", 3000))
    {
        Serial.println("SIM PIN READY");
        SIM_PIN_SET = true;
        return;
    }

    else
    {
        Serial.println("SIM PIN NOT SET");
        return;
        // ToDO:Set PIN
    }
}

bool is_SIMCID_valid() // ! Seems to be returning true even when there is "ERROR" in response
{
    char qccid[21];

    char AT_response[255] = {};

    char expected_reply[] = "+QCCID: ";

    get_raw_response("AT+QCCID\0", AT_response, 255, true, 5000);

    if (extractText(AT_response, expected_reply, qccid, 21, '\r') && strlen(qccid) == 20)
    {
        strcpy(SIM_CCID, qccid);
        SIM_AVAILABLE = true;
        return SIM_AVAILABLE;
    }
    else
    {

        return false;
    }
}

// Similar to FONA enableGPRS() but quicker because APN setting are not configured as it is configured during GSM_init()
bool GPRS_init()
{

    String err = "";

#ifdef QUECTEL
    Serial.println("Quectel GPRS init...");

    int timeout = 5000;
    Serial.println("Configuring PDP context ");
    bool PDP_config = false;
    while (timeout > 0)
    {
        // PDP_config = fona.sendCheckReply(F("AT+QICSGP=1,1"), F("OK"));
        PDP_config = sendAndCheck("AT+QICSGP=1,1\0", "OK");

        if (PDP_config)
        {
            Serial.println("PDP context set");
            break;
        }
        Serial.print(".");
        timeout -= 1000;
        delay(2000);
    }

    if (!PDP_config)
    {
        err = "Failed to config GPRS PDP context";
        GSM_INIT_ERROR = err;
        Serial.println(err);
        return false;
    }

    // Check CGATT status
    Serial.println("\nChecking CGATT Status..");
    // fona.sendParseReply(F("AT+CGATT?"), F("+CGATT:"), &CGATT_status, ' ', 1);
    CGATT_status = GPRS_status();
    Serial.println("CGATT_status: " + (String)CGATT_status);

    if (CGATT_status == 1)
    {
        GPRS_CONNECTED = true;
        GPRS_INIT_FAIL_COUNT = 0;
    }

    // Attach CGATT
    else
    {

        if (activateGPRS())
        {
            delay(2000);
            CGATT_status = getNumber("AT+CGATT?\0", "+CGATT: ", 0, 1);
            if (CGATT_status == 1)
                GPRS_CONNECTED = true;
        }
        else
        {
            Serial.println("CGATT status set to: " + (String)CGATT_status); // !! sometimes not reached when using if statement. delay needed
        }
    }

    // if (!fona.sendCheckReply(F("AT+QIACT=1"), F("OK"), 3000))
    // {
    //     err = "Failed to activate GPRS PDP context";
    //     GSM_INIT_ERROR = err;
    //     Serial.println(err);
    //     GPRS_CONNECTED = false;
    //     return GPRS_CONNECTED;
    // }

#else
    // "AT+SAPBR=1,1"
    // "AT+QCFG=\"gprsattach\",1"
#endif

    if (!GPRS_CONNECTED)
    {
        GPRS_INIT_FAIL_COUNT += 1;
    }
    return GPRS_CONNECTED;
}

void GSM_soft_reset()
{
    // #ifdef QUECTEL
    //     // ! Observation per v1 of Quectel PCB is that it POWERS BACK ON immediately after sending POWER DOWN command
    //     if (fona.sendCheckReply(F("AT+QPOWD"), F("POWERED DOWN")))
    //     {
    //         Serial.println("Restarting QUECTEL GSM");
    //         delay(10000); // Give module enough time to register to network
    //     }
    //     else
    //     {
    //         Serial.println("Failed to power down Quectel module");
    //     }

    // #else
    fona.enableGPRS(false); // basically shut down GPRS service

    if (!fona.sendCheckReply(F("AT+CFUN=1,1"), F("OK")))
    {
        Serial.println("Soft resetting GSM with full functionality failed!");
        return;
    }
    Serial.println("Soft resetting the GSM module...");
    delay(30000); // wait for GSM to warm up
    // #endif
}

/***
 * ? Called 3 times. Review the impelementation of this
 * Todo: Change implementation to shut down GSM and then call GSM_init();
 *
 *
 ***/
void restart_GSM()
{
    Serial.println("Restarting GSM");
    //! The AQ PCB board has the GSM reset physically connected to the ESP chip
    // GSM_soft_reset();
    // if (!fona.begin(*fonaSerial))
    // {
    //     Serial.println("Couldn't find GSM");
    //     return;
    // }

    if (!GSM_init(fonaSerial))
    {
        Serial.println("GSM not fully configured");
        Serial.print("Failure point: ");
        Serial.println(GSM_INIT_ERROR);
        Serial.println();
    }
}

void enableGPRS()
{
    // fona.setGPRSNetworkSettings(FONAFlashStringPtr(gprs_apn), FONAFlashStringPtr(gprs_username), FONAFlashStringPtr(gprs_password));

    int retry_count = 0;
    while ((fona.GPRSstate() != 0) && (retry_count < 40))
    {
        delay(3000);
        fona.enableGPRS(true);
        retry_count++;
    }

    fona.enableGPRS(true);
}

void disableGPRS()
{
    fona.enableGPRS(false);
    GPRS_CONNECTED = false;
}

/*****************************************************************
flushSerial
*****************************************************************/
void flushSerial()
{
    Serial.println("Flushing fona serial..\n############");
    while (fonaSS.available())
    {

        Serial.print(fonaSS.read());
    }
    Serial.println("##################");
}

/// @brief Easy implementation of Quectel HTTP functionality
/// @param url url for http request sans protocol
/// @param headers array of request headers
/// @param header_size size of the headers array
/// @param data post body data
/// @param data_length length of the data
void QUECTEL_POST(const char *url, char headers[][40], int header_size, const char *data, size_t data_length)
{
    /* SETTING request headers
    ! Headers are sent in two formats
    1. Format 0: headers are sent before post body
    2. Format 1: headers are sent as part of the body
    */

    // Using format 0

    // Config URL
    // String HTTP_SETUP = "AT+QHTTPURL=" + String(strlen(url), DEC) + ",10,60";

    char HTTP_CFG[128] = {};
    strcpy(HTTP_CFG, "AT+QHTTPCFG=\"url\",\"http://"); // protocol must be set before URL
    strcat(HTTP_CFG, url);
    strcat(HTTP_CFG, "\"");
    // String HTTP_CFG = "AT+QHTTPCFG=\"url\",\"http://" + String(url) + "\"";
    Serial.print("Quectel URL config: ");
    Serial.println(HTTP_CFG);
    sendAndCheck(HTTP_CFG, "OK", 2000);
    // fona.sendCheckReply(F("AT+QHTTPCFG=\"contextid\",1"), F("OK"));      // set context id
    sendAndCheck("AT+QHTTPCFG=\"contextid\",1", "OK");
    // fona.sendCheckReply(F("AT+QHTTPCFG=\"requestheader\",0"), F("OK"));  // disable request headers
    sendAndCheck("AT+QHTTPCFG=\"requestheader\",0", "OK");
    // fona.sendCheckReply(F("AT+QHTTPCFG=\"responseheader\",1"), F("OK")); // enable response headers
    sendAndCheck("AT+QHTTPCFG=\"responseheader\",1", "OK");
    // fona.sendCheckReply(F("AT+QHTTPCFG=\"rspout/auto\",1"), F("OK"));    // enable auto response and "disable" HTTTPREAD
    sendAndCheck("AT+QHTTPCFG=\"rspout/auto\",1", "OK");

    for (int i = 0; i < header_size; i++)
    {
        memset(HTTP_CFG, 0, sizeof(HTTP_CFG));
        strcpy(HTTP_CFG, "AT+QHTTPCFG=\"header\",\"");
        strcat(HTTP_CFG, headers[i]);
        strcat(HTTP_CFG, "\"");
        if (sendAndCheck(HTTP_CFG, "OK"))
        {
            Serial.println("Header set successfully");
        }
        else
        {
            Serial.println("Failed to set header");
            return;
        }
    }

    char HTTP_POST_RESPONSE_STATUS[4];

    // POST data
    // HTTP_CFG = "AT+QHTTPPOST=" + String(data_length) + ",30,60";
    char http_post_prepare[32] = "AT+QHTTPPOST=";
    char data_len[4];
    itoa(data_length, data_len, 10);
    strcat(http_post_prepare, data_len);
    strcat(http_post_prepare, ",30,60");

    Serial.println(http_post_prepare);
    if (sendAndCheck(http_post_prepare, "CONNECT", 10000)) // Allow enough time to connect to HTTP(S) server
    {
        Serial.println("Posting gprs data..");
        get_http_response_status(data, HTTP_POST_RESPONSE_STATUS);
    }
    else
    {
        Serial.println("HTTP POST CONNECT FAIL");
        HTTPCFG_CONNECT_FAIL += 1;
        return;
    }

    if (strstr(HTTP_POST_RESPONSE_STATUS, "20"))
    {
        Serial.println("Requested processed successfully with status: " + (String)HTTP_POST_RESPONSE_STATUS);
    }
    else
    {
        Serial.println("Requested processing failed with status: " + (String)HTTP_POST_RESPONSE_STATUS);
        HTTP_POST_FAIL += 1;
    }
}

void SerialFlush()
{
    // Serial.flush();
    Serial.println("Flushing ESP serial..\n************");
    while (Serial.available())
    {
        Serial.print(Serial.read());
    }
    Serial.println("************");
}

void get_raw_response(const char *cmd, char *res_buff, size_t buff_size, bool fill_buffer, unsigned long timeout)
{

    flushSerial();
    // SerialFlush();
    memset(res_buff, '\0', buff_size);
    // Serial.println("Size of response buffer: " + (String)buff_size);
    size_t buff_pos = 0;
    Serial.print("Received Command in get raw: ");
    Serial.println(cmd);
    fona.println(cmd);
    unsigned long sendStartMillis = millis();
    do
    {
        if (buff_pos >= buff_size) // Check if buff is full
            break;

        while (fona.available())
        {

            if (buff_pos >= buff_size)
                break;
            res_buff[buff_pos] = fona.read();
            buff_pos++;
        }

        // // eat unsolicited result code "RDY"
        // if (strstr(res_buff, "RDY"))
        // {
        //     // reset buff
        //     memset(res_buff, '\0', buff_size);
        //     buff_pos = 0;
        //     Serial.println("Eating URC 'RDY'");
        // }

        delay(2);
    } while ((fill_buffer ? fill_buffer : strlen(res_buff) == 0) && (millis() - sendStartMillis < timeout));
    Serial.println("\n-------\r\nGSM RAW RESPONSE:");
    Serial.println(res_buff);
    Serial.println("-------");
}

/***
 @brief : Extract a piece of text matching the target from a char array
 @param input : The char array that contains the string to be parsed from
 @param target : Ocuurence of a particular string
 @param output : A char array to store extracted string
 @param _until : The first character matching to read from after finding occurence of the target
 @return
 ****/
bool extractText(char *input, const char *target, char *output, uint8_t output_size, char _until)
{

    const char *found_target = strstr(input, target);

    if (found_target != nullptr)
    {

        Serial.print("Substring found at position: ");
        Serial.println(found_target - input);

        // Find the start of the extraction point
        const char *start = found_target + strlen(target);

        // Find the end of the extraction point (the next comma by default)
        const char *end = strchr(start, _until);

        if (end != nullptr)
        {
            // Calculate the length of the text to be extracted
            size_t length = end - start;

            if (length < output_size)
            { // check for buffer overflow.
                strncpy(output, start, length);
                output[length] = '\0'; // Null-terminate the string
                return true;
            }
            else
            {
                Serial.println("Extracted piece of text longer than ouput size");
                return false;
            }
        }
    }
    Serial.println("Could not extact substring '" + (String)target + "' from the source");
    return false;
}

// extract an integer
int16_t getNumber(char *AT_cmd, char *expected_reply, uint8_t index_from, uint8_t length)
{

    int16_t num;

    char AT_response[255];
    size_t AT_res_size = sizeof(AT_response);

    char number[8];

    if (length > sizeof(number))
    {
        Serial.println("max length allowed is 8");
        return -1;
    }

    get_raw_response(AT_cmd, AT_response, AT_res_size);

    const char *found_target = strstr(AT_response, expected_reply);

    if (found_target == nullptr)
        return -1;

    // Find the start of desired extraction point
    const char *start = found_target + strlen(expected_reply);
    start += index_from; // E.g to extract 5 from +CREG: 0,5,7 will start from '+CREG: ' + 2 indices

    if (length < sizeof(number))
    {

        strncpy(number, start, length);
        number[length] = '\0';
    }

    Serial.print("Extracted number: ");
    Serial.println(number);

    num = atoi(number);
    return num;
}

/// @brief simple function to send AT command and check for expected reply
/// @param AT_cmd : AT command to send
/// @param expected_reply : expect reply from the AT command to contain this string
/// @return true if expected reply is found
bool sendAndCheck(const char *AT_cmd, const char *expected_reply, unsigned long timeout)
{
    char AT_response[255];
    size_t AT_res_size = sizeof(AT_response);

    get_raw_response(AT_cmd, AT_response, AT_res_size, true, timeout);

    if (strstr(AT_response, expected_reply))
    {
        return true;
    }

    return false;
}

void get_http_response_status(String data, char *HTTP_RESPONSE_STATUS)
{
    char HTTP_RESPONSE[255];
    size_t BUFFER_SIZE = sizeof(HTTP_RESPONSE);
    const char *data_copy = data.c_str();
    char gprs_data[strlen(data_copy)];
    strcpy(gprs_data, data_copy);
    get_raw_response(gprs_data, HTTP_RESPONSE, BUFFER_SIZE, true, 10000);

    // Check HTTP RESPONSE status
    const char *expected_reply = "+QHTTPPOST: 0,"; // Operartion successful

    if (extractText(HTTP_RESPONSE, expected_reply, HTTP_RESPONSE_STATUS, 4, ','))
    {

        Serial.print("Gotten http status code: ");
        Serial.println(HTTP_RESPONSE_STATUS);
    }
    else
    {
        Serial.println("Could not extract HTTP response status code");
        //? Maybe troubleshoot
    }
}

// Simple function to troubleshoot GSM //? More to be done
void troubleshoot_GSM()
{

    GSM_init(fonaSerial); // ! Use GSM soft reset if GSM reset pin is not connected

    register_to_network();

    GPRS_init();

    // RESET FLAGS
    HTTPCFG_CONNECT_FAIL = 0;
    HTTP_POST_FAIL = 0;
    GPRS_INIT_FAIL_COUNT = 0;
}

int8_t GPRS_status()
{

    int8_t status = getNumber("AT+CGATT?\0", "+CGATT: ", 0, 1);
    Serial.print("CGATT status: ");
    Serial.println(status);
    return status;
}

bool activateGPRS()
{
    if (GPRS_status() == 1)
    {
        Serial.println("GPRS already active");
        return true;
    }
    if (sendAndCheck("AT+CGATT=1", "OK"))
    {

        return true;
    }
    else
    {
        Serial.println("Failed to enable GPRS");
        return false;
    }
}

bool deactivateGPRS()
{

    if (GPRS_status() == 0)
    {
        Serial.println("GPRS already inactive");
        return true;
    }
    else
    {
        if (sendAndCheck("AT+CGATT=0", "OK"))
        {
            // query GPRS status
            GPRS_status();
            return true;
        }
        else
        {
            Serial.println("Failed to disable GPRS");
            return false;
        }
        return true;
    }
}

/**
  @brief: Query the real time clock (RTC) of the module.
  @param time : The format is "yy/MM/dd,hh:mm:ss±zz",indicating year (two last digits),
                month, day, hour, minutes, seconds and
                time zone (indicates the difference, expressed in quarters of an hour, between the local time and GMT; range: -48 to +56).
                E.g. May 6th, 1994, 22:10:00 GMT+2 hours equals to "94/05/06,22:10:00+08"
  @return bool : true if time is successfully extracted or false if otherwise
*/
bool getNetworkTime(char *time)
{

    char AT_response[64];
    size_t AT_res_size = sizeof(AT_response);
    char time_buff[23];
    uint8_t retries = 0;

    get_raw_response("AT+CCLK?\0", AT_response, AT_res_size, false, 5000);

    while (!extractText(AT_response, "+CCLK: \"", time_buff, 32, '\"') && retries < 10)
    {
        get_raw_response("AT+CCLK?\0", AT_response, AT_res_size, false, 5000);
        retries++;
        delay(1000);
    }

    if (strlen(time_buff) > 0)
    {
        strcpy(time, time_buff);
        return true;
    }
    else
    {
        return false;
    }
}

// Testing POST data
// http://staging.api.sensors.africa/v1/push-sensor-data/

// POST /v1/push-sensor-data/\r\nHost: http://staging.api.sensors.africa\r\nAccept: */*\r\nUser-Agent: QUECTEL EC200\r\nContent-Type: application/json\r\nX-Sensor: esp8266-15355455\r\nX-PIN: 1\r\nContent-Length: 385\r\n\r\n{"software_version": "NRZ-2020-129", "sensordatavalues":[{"value_type":"P0","value":"7.80"},{"value_type":"P1","value":"10.50"},{"value_type":"P2","value":"13.40"}]}\r\n
// data length 252

// Accept: */*\r\nUser-Agent: QUECTEL EC200\r\nContent-Type: application/json\r\nX-Sensor: esp8266-15355455\r\nX-PIN: 1\r\nContent-Length: 165\r\n\r\n{"software_version": "NRZ-2020-129", "sensordatavalues":[{"value_type":"P0","value":"7.80"},{"value_type":"P1","value":"10.50"},{"value_type":"P2","value":"13.40"}]}\r\n
/// 1234

// AT commands sequence

// AT+CGATT=1
// AT+QICSGP=1,1,"safaricom","saf","data"
// AT+QIACT=1
// AT+QIACT?
// AT+QHTTPCFG="contextid",1
// AT+QHTTPCFG="requestheader",1
// AT+QHTTPCFG="responseheader",1
// AT+QHTTPURL=54,30,60
// http://staging.api.sensors.africa/v1/push-sensor-data/
// AT+QHTTPPOST=385,30,60
// AT+QHTTPREAD
