HardwareSerial GSMSerial(2);

enum RST_SEQ
{
    HIGH_LOW_HIGH,
    LOW_HIGH_LOW
};

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

enum NetMode // Quectel Modem
{
    AUTO = 0,
    _2G = 1,
    _4G = 3,
};
NetMode current_network = NetMode::AUTO;

enum NetRegStatus // 0-5
{
    NOT_REGISTERED,
    REGISTERED_TO_HOME_NETWORK,
    SEARCHING,
    REGISTRATION_DENIED,
    UNKNOWN,
    REGISTERED_ROAMING
};

static const char *const NET_STATUS_VERBOSE[] = {
    "Not registered to network",
    "Registered to home network",
    "Searching for network",
    "Network registration denied",
    "Network registration status unknown",
    "Registered to roaming network"

};

/**** Function Declacrations **/
bool GSM_init();
bool register_to_network();
static void unlock_pin(char *PIN);
void SIM_PIN_Setup();
bool is_SIMCID_valid();
bool GPRS_init();
void GSM_soft_reset();
void restart_GSM();
bool activateGPRS();
bool deactivateGPRS();
int8_t GPRS_status();
void flushSerial();
void SerialFlush();
void QUECTEL_POST(const char *url, char headers[][40], int header_size, const char *data, size_t data_length, uint8_t &response_status);
bool extractText(char *input, const char *target, char *output, uint8_t output_size, char _until); // ? should go to utils
void get_raw_response(const char *cmd, char *res_buff, size_t buff_size, bool wait_timeout = false, unsigned long timeout = 3000);
int16_t getNumber(const char *AT_cmd, const char *expected_reply, uint8_t index_from, uint8_t length);
void get_http_response_status(String data, char *HTTP_RESPONSE_STATUS);
bool sendAndCheck(const char *AT_cmd, const char *expected_reply = "OK", unsigned long timeout = 10000);
bool sendAndCheck(const char *AT_cmd, const char *expected_reply, String &response,
                  unsigned long timeout = 10000);
bool waitForReply(const char *expectedReply, unsigned long timeout);
bool waitForReply(const char *expectedReply, String &buffer, unsigned long timeout);
bool waitForURC(const char *urcPrefix, char *response, size_t responseLen, unsigned long timeout);
bool GSM_Serial_begin();
bool getNetworkTime(char *time);
void GSMreset(RST_SEQ seq, uint8_t timing_delay = 120);
void http_preconfig();
void GSM_sleep();
void troubleshoot_GSM();
String getNetworkName();
int8_t getSignalStrength();
String getNetworkBand();
bool setNetworkMode(NetMode mode);
void cycleNetworkMode();

bool GSM_init()
{

    String error_msg = "";

    Serial.println("Restarting GSM...");

#ifdef GSM_RST_PIN

    GSMreset(RST_SEQ::LOW_HIGH_LOW);
#else
    GSM_soft_reset();

#endif

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

    return true;
}

bool setNetworkMode(NetMode mode)

{
    char setnetmode[24] = "AT+QCFG=\"nwscanmode\",";
    char _mode[1];
    itoa(mode, _mode, 10);

    strcat(setnetmode, _mode);

    char mode_str[8];
    switch (mode)
    {
    case (NetMode::_2G):
        strcpy(mode_str, "2G");
        break;
    case (NetMode::_4G):
        strcpy(mode_str, "4G");
        break;
    case (NetMode::AUTO):
        strcpy(mode_str, "AUTO");
        break;
    }

    Serial.print("Setting network mode to: ");
    Serial.println(mode_str);

    if (!sendAndCheck(setnetmode, "OK", 2000))
    {
        Serial.print("Failed to set network mode: ");
        Serial.println(mode_str);
        return false;
    }
    delay(1000);
    current_network = mode;
    return true;
}

void cycleNetworkMode()
{
    bool set_mode = setNetworkMode(current_network);
    // Cycle to the next network mode (AUTO -> 2G -> 4G -> AUTO)
    switch (current_network)
    {
    case (NetMode::AUTO):
        current_network = NetMode::_2G;
        break;
    case (NetMode::_2G):
        current_network = NetMode::_4G;
        break;
    case (NetMode::_4G):
        current_network = NetMode::AUTO;
        break;
    default:
        current_network = NetMode::AUTO;
        break;
    }

    if (!set_mode) // Attempt setting the net mode
        cycleNetworkMode();
}

bool register_to_network()
{

    String error_msg = "";
    bool registered_to_network = false;
    int retry_count = 0;
    int8_t status;
    cycleNetworkMode();
    if (!sendAndCheck("AT+CREG=1\0", "OK"))
    {
        Serial.println("Manual network registration failed.");
    }
    while (!registered_to_network && retry_count < 20)
    {
        status = getNumber("AT+CREG?\0", "+CREG: ", 2, 1);

        if (status == NetRegStatus::REGISTERED_TO_HOME_NETWORK || status == NetRegStatus::REGISTERED_ROAMING)
        {
            registered_to_network = true;
            break;
        }

        else
        {
            Serial.print(NET_STATUS_VERBOSE[status]);
        }

        retry_count++;
        delay(3000);
    }

    if (!registered_to_network)
    {
        error_msg = NET_STATUS_VERBOSE[status];
        GSM_INIT_ERROR = error_msg;
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

    sendAndCheck("AT+COPS?", "OK");

    return true;
}

// static void unlock_pin(char *PIN)
// {

//     // Attempt to SET PIN if not empty
//     Serial.print("GSM CONFIG SET PIN: ");
//     Serial.println(PIN);
//     // Serial.print("Length of PIN");
//     Serial.println(strlen(PIN));
//     if (strlen(PIN) > 1)
//     {
//         SIM_PIN_SET = true;
//     }
// }

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
    String AT_response = "";

    char expected_reply[] = "+QCCID: ";
    if (!sendAndCheck("AT+QCCID\0", "OK", AT_response))
        return false;

    if (extractText((char *)AT_response.c_str(), expected_reply, qccid, 21, '\r') && strlen(qccid) == 20)
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
    CGATT_status = GPRS_status();
    // Serial.println("CGATT_status: " + (String)CGATT_status);

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

    //? QIACT

#else
    // "AT+SAPBR=1,1"
    // "AT+QCFG=\"gprsattach\",1"
#endif

    if (!GPRS_CONNECTED)
    {
        GPRS_INIT_FAIL_COUNT += 1;
    }
    else
    {
        http_preconfig();
    }
    return GPRS_CONNECTED;
}

void GSM_soft_reset()
{

    deactivateGPRS();

    if (!sendAndCheck("AT+CFUN=1,1", "OK"))
    {
        Serial.println("Soft resetting GSM with full functionality failed!");
        return;
    }
    Serial.println("Soft resetting the GSM module...");
    delay(30000); // wait for GSM to warm up
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

    if (!GSM_init())
    {
        Serial.println("GSM not fully configured");
        Serial.print("Failure point: ");
        Serial.println(GSM_INIT_ERROR);
        Serial.println();
    }
}

/*****************************************************************
flushSerial
*****************************************************************/
void flushSerial()
{
    // Serial.println("Flushing GSM serial..\n############");
    while (GSMSerial.available())
    {
        char c = GSMSerial.read();
        // Serial.print(); //? Only useful for debugging
    }
    // Serial.println("\n##################");
}

/// @brief Preconfigure HTTP settings
/// @details This function sets the HTTP configuration for the Quectel module.
void http_preconfig()
{
    sendAndCheck("AT+QHTTPCFG=\"contextid\",1", "OK");
    sendAndCheck("AT+QHTTPCFG=\"requestheader\",0", "OK");
    sendAndCheck("AT+QHTTPCFG=\"responseheader\",1", "OK");
    sendAndCheck("AT+QHTTPCFG=\"rspout/auto\",0", "OK");
}

/// @brief Easy implementation of Quectel HTTP functionality
/// @param url url for http request sans protocol
/// @param headers array of request headers
/// @param header_size size of the headers array
/// @param data post body data
/// @param data_length length of the data
/// @param response_status integer address to store the response status
void QUECTEL_POST(const char *url, char headers[][40], int header_size, const char *data, size_t data_length, uint8_t &response_status)
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
    Serial.print("Quectel URL config: ");
    Serial.println(HTTP_CFG);

    sendAndCheck(HTTP_CFG, "OK", 2000);

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
        response_status = atoi(HTTP_POST_RESPONSE_STATUS);
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

void get_raw_response(const char *cmd, char *res_buff, size_t buff_size, bool wait_timeout, unsigned long timeout)
{

    flushSerial();
    // SerialFlush();
    memset(res_buff, '\0', buff_size);
    // Serial.println("Size of response buffer: " + (String)buff_size);
    size_t buff_pos = 0;
    Serial.print("Received Command in get raw: ");
    Serial.println(cmd);
    GSMSerial.println(cmd);
    unsigned long sendStartMillis = millis();
    do
    {
        if (buff_pos >= buff_size) // Check if buff is full
            break;

        while (GSMSerial.available())
        {

            if (buff_pos >= buff_size)
                break;
            res_buff[buff_pos] = GSMSerial.read();
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
    } while ((wait_timeout || (buff_pos == 0)) && (millis() - sendStartMillis < timeout));
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

        // Serial.print("Substring found at position: ");
        // Serial.println(found_target - input);

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
int16_t getNumber(const char *AT_cmd, const char *expected_reply, uint8_t index_from, uint8_t length)
{

    int16_t num;

    // char AT_response[255];
    // size_t AT_res_size = sizeof(AT_response);

    char number[8];

    if (length > sizeof(number))
    {
        Serial.println("max length allowed is 8");
        return -1;
    }

    // get_raw_response(AT_cmd, AT_response, AT_res_size);
    String AT_response = "";

    if (!sendAndCheck(AT_cmd, "OK", AT_response))
        return -1;

    const char *found_target = strstr(AT_response.c_str(), expected_reply);

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
    // Debug
    // Serial.print("Extracted number: ");
    // Serial.println(number);

    num = atoi(number);
    return num;
}

/// @brief simple function to send AT command and check for expected reply
/// @param AT_cmd : AT command to send
/// @param expected_reply : expect reply from the AT command to contain this string
/// @return true if expected reply is found
bool sendAndCheck(const char *AT_cmd, const char *expected_reply, unsigned long timeout)
{
    flushSerial();
    GSMSerial.println(AT_cmd);
    return waitForReply(expected_reply, timeout);
}

bool sendAndCheck(const char *AT_cmd, const char *expected_reply, String &response,
                  unsigned long timeout)
{
    flushSerial();
    GSMSerial.println(AT_cmd);

    return waitForReply(expected_reply, response, timeout);
}

bool waitForReply(const char *expectedReply, unsigned long timeout)
{
    unsigned long start = millis();
    String buffer = "";

    while (millis() - start < timeout)
    {
        while (GSMSerial.available())
        {
            char c = GSMSerial.read();
            buffer += c;

            if (buffer.indexOf(expectedReply) >= 0)
            {
                // Serial.println(buffer);
                return true;
            }

            // Maintain small buffer size
            if (buffer.length() > 256)
            {
                buffer = buffer.substring(buffer.length() - 128);
            }
        }
    }
    return false;
}

bool waitForReply(const char *expectedReply, String &buffer, unsigned long timeout)
{
    unsigned long start = millis();
    buffer = "";

    while (millis() - start < timeout)
    {
        while (GSMSerial.available())
        {
            char c = GSMSerial.read();
            buffer += c;

            if (buffer.indexOf(expectedReply) >= 0)
            {
                // Serial.println(buffer);
                return true;
            }

            if (buffer.length() > 256)
            {
                buffer = buffer.substring(buffer.length() - 128);
            }
        }
    }
    return false;
}

bool waitForURC(const char *urcPrefix, char *response,
                size_t responseLen, unsigned long timeout)
{
    unsigned long start = millis();
    String buffer = "";

    while (millis() - start < timeout)
    {
        while (GSMSerial.available())
        {
            char c = GSMSerial.read();
            buffer += c;

            if (c == '\n')
            {
                if (buffer.indexOf(urcPrefix) >= 0)
                {
                    strncpy(response, buffer.c_str(), responseLen - 1);
                    response[responseLen - 1] = '\0';
                    return true;
                }
                buffer = "";
            }

            if (buffer.length() > 256)
            {
                buffer = "";
            }
        }
    }
    return false;
}

void get_http_response_status(String data, char *HTTP_RESPONSE_STATUS)
{
    // char HTTP_RESPONSE[255];
    // size_t BUFFER_SIZE = sizeof(HTTP_RESPONSE);
    const char *data_copy = data.c_str();
    char gprs_data[strlen(data_copy)];
    strcpy(gprs_data, data_copy);

    // get_raw_response(gprs_data, HTTP_RESPONSE, BUFFER_SIZE, true, 10000);

    // Check HTTP RESPONSE status
    const char *expected_reply = "+QHTTPPOST: 0,"; // Operartion successful
    sendAndCheck(gprs_data);

    char qurc[32];
    if (!waitForURC("+QHTTPPOST: ", qurc, sizeof(qurc), 10000)) //? notice space after colon
    {
        Serial.println("HTTP POST QURC not received!");
        return;
    }

    if (extractText(qurc, expected_reply, HTTP_RESPONSE_STATUS, 4, ','))
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

    GSM_init(); // ! Use GSM soft reset if GSM reset pin is not connected

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
    // Serial.print("CGATT status: ");
    // Serial.println(status);
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

    // char AT_response[64];
    // size_t AT_res_size = sizeof(AT_response);
    String AT_response = "";
    char time_buff[23] = {};
    uint8_t retries = 0;

    // get_raw_response("AT+CCLK?\0", AT_response, AT_res_size, false, 5000);
    sendAndCheck("AT+CCLK?", "OK", AT_response);

    while (!extractText((char *)AT_response.c_str(), "+CCLK: \"", time_buff, 32, '\"') && retries < 10)
    {
        sendAndCheck("AT+CCLK?", "OK", AT_response);
        retries++;
        delay(1000);
    }

    String time_str = String(time_buff);

    if (time_str.charAt(2) == '/' && time_str.charAt(5) == '/' && time_str.charAt(8) == ',' && time_str.charAt(11) == ':' && time_str.charAt(14) == ':')
    {

        // Serial.println("Time length: " + (String)strlen(time_buff));
        // Serial.println("Time buffer: " + (String)time_buff);
        // Serial.println("Time buffer size: " + (String)sizeof(time_buff));

        strcpy(time, time_buff);
        return true;
    }
    else
    {
        return false;
    }
}

String getNetworkName()
{

    // char AT_response[255];
    // size_t AT_res_size = sizeof(AT_response);
    String AT_response = "";

    const char AT_cmd[] = "AT+QSPN";
    char NetworkName[64];

    // get_raw_response(AT_cmd, AT_response, AT_res_size, true, 300);
    if (!sendAndCheck(AT_cmd, "OK", AT_response, 300))
        return "";

    if (extractText((char *)AT_response.c_str(), "+QSPN: \"", NetworkName, 64, '"'))
    {
        NETWORK_NAME = String(NetworkName);
        return NETWORK_NAME;
    };

    NETWORK_NAME = "";
    return NETWORK_NAME;
}

int8_t getSignalStrength()
{
    // char AT_response[64];
    // size_t AT_res_size = sizeof(AT_response);
    String AT_response = "";
    const char AT_cmd[] = "AT+CSQ";
    char rssi[4];
    if (!sendAndCheck(AT_cmd, "OK", AT_response, 300))
        return 99;

    // get_raw_response(AT_cmd, AT_response, AT_res_size, true, 300);
    if (extractText((char *)AT_response.c_str(), "+CSQ: ", rssi, sizeof(rssi), ','))
    {
        return atoi(rssi);
    };
    return 99;
}

String getNetworkBand()
{
    // char AT_response[64];
    // size_t AT_res_size = sizeof(AT_response);

    const char AT_cmd[] = "AT+QNWINFO";
    char band[64];
    String AT_response = "";
    if (!sendAndCheck(AT_cmd, "OK", AT_response, 300))
        return "";

    // get_raw_response(AT_cmd, AT_response, AT_res_size, true, 300);

    // Extract text between second and third comma
    // Expected response example: +QNWINFO: "FDD LTE","63902","LTE BAND 3",1650
    if (extractText((char *)AT_response.c_str(), "+QNWINFO: \"", band, sizeof(band), '\n'))
    {
        // ToDo: Extract Access Technology: Particulary interested in "NO SERVICE" as part of response
        // Find the second occurrence of comma and extract from there
        const char *start = strchr((char *)AT_response.c_str(), ',');
        if (start != nullptr)
        {
            start = strchr(start + 1, ',');
            if (start != nullptr)
            {
                start++; // Move past the comma
                // Skip leading quote if present
                if (*start == '"')
                    start++;

                const char *end = strchr(start, '"');
                if (end != nullptr)
                {
                    size_t length = end - start;
                    if (length < sizeof(band))
                    {
                        strncpy(band, start, length);
                        band[length] = '\0';
                        return String(band);
                    }
                }
            }
        }
    }

    return "";
}

bool GSM_Serial_begin()
{
    pinMode(QUECTEL_PWR_KEY, OUTPUT);
    digitalWrite(QUECTEL_PWR_KEY, HIGH);

    GSMSerial.begin(115200, SERIAL_8N1, MCU_RXD, MCU_TXD);

    bool comm_init = false;

    int16_t timeout = 30000;

    Serial.println("Attempting to initate comms with GSM module");

    while (millis() < timeout)
    {
        while (GSMSerial.available())
            GSMSerial.read();
        if (sendAndCheck("AT", "OK"))
        {
            comm_init = true;
            Serial.println("GSM module found!");
            break;
        }
    }
    if (!comm_init)
    {
        return false;
    }

// debug
#ifdef GSM_DEBUG
    sendAndCheck("ATE1", "OK");
    sendAndCheck("AT+CMEE=2", "OK");
#else
    sendAndCheck("ATE0", "OK");
    sendAndCheck("AT+CMEE=1", "OK");
#endif
    sendAndCheck("ATI", "OK");

    // Set automatic timezone and update Locate time to RTC
    sendAndCheck("AT+CTZU=3", "OK");

    return comm_init;
}

/// @brief Reset GSM module
/// @param seq: Sequence to to toggle reset pin to trigger a restart
/// @param timing_delay : Timing function for the reset to happen
void GSMreset(RST_SEQ seq, uint8_t timing_delay)
{

    pinMode(GSM_RST_PIN, OUTPUT);

    if (seq == LOW_HIGH_LOW)
    {
        digitalWrite(GSM_RST_PIN, LOW);
        delay(timing_delay);
        digitalWrite(GSM_RST_PIN, HIGH);
        delay(timing_delay);
        digitalWrite(GSM_RST_PIN, LOW);
    }
    else if (seq == HIGH_LOW_HIGH)
    {
        digitalWrite(GSM_RST_PIN, HIGH);
        delay(timing_delay);
        digitalWrite(GSM_RST_PIN, LOW);
        delay(timing_delay);
        digitalWrite(GSM_RST_PIN, HIGH);
    }

    delay(30000); // Allow enough time for GSM to warm up
}

void GSM_sleep()
{

    if (sendAndCheck("AT+QSCLK=2", "OK"))
    {
        Serial.println("GSM module is now in sleep mode. Will only wake up if data is sent on the serial port");
    }
    else
    {
        Serial.println("Failed to put GSM module in sleep mode");
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
