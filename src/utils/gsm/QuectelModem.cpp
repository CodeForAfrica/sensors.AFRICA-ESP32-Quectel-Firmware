#include "QuectelModem.h"

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace gsm
{

    namespace
    {
        bool isRegistered(int status)
        {
            return status == static_cast<int>(NetworkRegistrationStatus::RegisteredHome) ||
                   status == static_cast<int>(NetworkRegistrationStatus::RegisteredRoaming);
        }

        NetworkMode nextNetworkMode(NetworkMode mode)
        {
            switch (mode)
            {
            case NetworkMode::Auto:
                return NetworkMode::G2;
            case NetworkMode::G2:
                return NetworkMode::G4;
            case NetworkMode::G4:
            default:
                return NetworkMode::Auto;
            }
        }
    } // namespace

    QuectelModem::QuectelModem(AtClient &atClient, Stream &logger, const SerialConfig &serialConfig)
        : at_(atClient), logger_(logger), serialConfig_(serialConfig), gnss_(atClient), fileSystem_(atClient)
    {
    }

    ModemState &QuectelModem::state()
    {
        return state_;
    }

    const ModemState &QuectelModem::state() const
    {
        return state_;
    }

    FailureCounters &QuectelModem::failures()
    {
        return failures_;
    }

    const FailureCounters &QuectelModem::failures() const
    {
        return failures_;
    }

    bool QuectelModem::beginSerial(HardwareSerial &serial)
    {
        if (&serial != &at_.serial())
            return false;

        state_.connected = false;
        // These are board-side GPIO levels. The tested reset driver is idle LOW.
        if (serialConfig_.resetPin >= 0)
        {
            const bool idleHigh = serialConfig_.resetSequence == ResetSequence::HighLowHigh;
            digitalWrite(serialConfig_.resetPin, idleHigh ? HIGH : LOW);
            pinMode(serialConfig_.resetPin, OUTPUT);
        }

        if (serialConfig_.rxPin >= 0 && serialConfig_.txPin >= 0)
            serial.begin(serialConfig_.baudRate, SERIAL_8N1, serialConfig_.rxPin, serialConfig_.txPin);
        else
            serial.begin(serialConfig_.baudRate);

        delay(serialConfig_.serialWarmupMs);

        // Do not toggle PWRKEY on an already responsive modem. An invalid binary
        // session requires reset recovery in initialize(), not more UART commands.
        if (!at_.synchronized())
            return false;
        if (waitForReady(1000))
        {
            state_.connected = true;
            return true;
        }

        if (serialConfig_.powerKeyPin >= 0)
        {
            // Preserve the board-tested startup sequence, including its final
            // held-HIGH level. Do not substitute modem-pin PWRKEY levels here.
            pinMode(serialConfig_.powerKeyPin, OUTPUT);
            digitalWrite(serialConfig_.powerKeyPin, LOW);
            delay(500);
            digitalWrite(serialConfig_.powerKeyPin, HIGH);
            delay(2500);
            digitalWrite(serialConfig_.powerKeyPin, HIGH);
            delay(serialConfig_.serialWarmupMs);
        }

        logger_.println("Attempting to initiate comms with GSM module");
        state_.connected = waitForReady();
        if (!state_.connected)
            logger_.println("Failed to establish communication with GSM module");
        return state_.connected;
    }

    bool QuectelModem::waitForReady(unsigned long timeoutMs)
    {
        if (!at_.synchronized())
            return false;

        const unsigned long start = millis();
        while (true)
        {
            const unsigned long elapsed = millis() - start;
            if (elapsed >= timeoutMs)
                break;
            const unsigned long remaining = timeoutMs - elapsed;
            if (at_.sendCommand("AT", "OK", remaining < 1000UL ? remaining : 1000UL))
                return true;
            delay(250);
        }
        return false;
    }

    void QuectelModem::clearConnectionState()
    {
        // Keep configured SIM PIN and accumulated failure counters across reboot.
        state_.connected = false;
        state_.simAvailable = false;
        state_.gprsConnected = false;
        state_.simPinSet = false;
        state_.simUsable = false;
        state_.cgattStatus = 0;
        state_.simCcid[0] = '\0';
        state_.networkName = "";
        state_.initError = "";
        failures_.mqttStatus = MqttConnectionStatus::Disconnected;
        failures_.httpPostResponseStatus = 0;
    }

    bool QuectelModem::configureCommands()
    {
        // Reapply volatile settings after reboot, not before it.
        return at_.sendCommand(serialConfig_.debugEnabled ? "ATE1" : "ATE0", "OK") &&
               at_.sendCommand(serialConfig_.debugEnabled ? "AT+CMEE=2" : "AT+CMEE=1", "OK") &&
               at_.sendCommand("ATI", "OK") &&
               at_.sendCommand("AT+CTZU=3", "OK") &&
               at_.sendCommand("AT&W", "OK");
    }

    bool QuectelModem::initialize()
    {
        logger_.println("Initializing GSM with software reset...");

        if (!softReset())
        {
            logger_.println("Software reset unavailable or failed; attempting hardware recovery");
            if (!hardwareReset(serialConfig_.resetSequence))
            {
                state_.initError = serialConfig_.resetPin < 0
                                       ? "Software reset failed; no hardware reset pin configured"
                                       : "Modem did not respond after hardware reset";
                logger_.println(state_.initError);
                return false;
            }
        }

        if (!configureCommands())
        {
            state_.initError = "Could not configure modem after reset";
            state_.connected = false;
            logger_.println(state_.initError);
            return false;
        }

        if (!isSimCcidValid())
        {
            state_.initError = "Could not get SIM CID";
            logger_.println(state_.initError);
            state_.connected = false;
            return false;
        }

        state_.simUsable = true;
        state_.connected = true;
        return true;
    }

    bool QuectelModem::registerToNetwork()
    {
        bool registered = false;

        if (!at_.sendCommand("AT+CREG=1", "OK"))
            logger_.println("Manual network registration failed.");

        for (uint8_t retry = 0; retry < 20; retry++)
        {
            int8_t status = static_cast<int8_t>(at_.readNumber("AT+CREG?", "+CREG: ", 2, 1));
            if (isRegistered(status))
            {
                registered = true;
                logger_.print(registrationStatusText(status));
                logger_.print(": ");
                logger_.println(networkName());
                break;
            }

            logger_.println(registrationStatusText(status));
            delay(3000);
        }

        if (!registered)
            failures_.networkRegistrationFailures++;

        return registered;
    }

    void QuectelModem::setupSimPin()
    {
        if (at_.sendCommand("AT+CPIN?", "+CPIN: READY", 3000))
        {
            logger_.println("SIM PIN READY");
            state_.simPinSet = true;
            return;
        }

        logger_.println("SIM PIN NOT SET");
        state_.simPinSet = false;
    }

    bool QuectelModem::isSimCcidValid()
    {
        char qccid[21] = {};
        String response;

        if (!at_.sendCommand("AT+QCCID", "OK", response))
            return false;

        if (AtClient::extractText(response.c_str(), "+QCCID: ", qccid, sizeof(qccid), '\r') && strlen(qccid) == 20)
        {
            strcpy(state_.simCcid, qccid);
            state_.simAvailable = true;
            return true;
        }

        state_.simAvailable = false;
        state_.simCcid[0] = '\0';
        return false;
    }

    bool QuectelModem::initializeGprs()
    {
        logger_.println("Quectel GPRS init...");
        state_.gprsConnected = false;

        int timeoutMs = 5000;
        bool pdpConfigured = false;
        logger_.println("Configuring PDP context ");
        while (timeoutMs > 0)
        {
            pdpConfigured = at_.sendCommand("AT+QICSGP=1,1", "OK");
            if (pdpConfigured)
            {
                logger_.println("PDP context set");
                break;
            }

            logger_.print(".");
            timeoutMs -= 1000;
            delay(2000);
        }

        if (!pdpConfigured)
        {
            state_.initError = "Failed to config GPRS PDP context";
            logger_.println(state_.initError);
            failures_.gprsInitFailures++;
            return false;
        }

        logger_.println("\nChecking CGATT Status..");
        state_.cgattStatus = gprsStatus();

        if (state_.cgattStatus == 1)
        {
            state_.gprsConnected = true;
            failures_.gprsInitFailures = 0;
        }
        else if (activateGprs())
        {
            delay(2000);
            state_.cgattStatus = gprsStatus();
            state_.gprsConnected = state_.cgattStatus == 1;
        }
        else
        {
            logger_.print("CGATT status set to: ");
            logger_.println(state_.cgattStatus);
        }

        if (!state_.gprsConnected)
        {
            logger_.println("Failed to init GPRS");
            failures_.gprsInitFailures++;
        }
        else
        {
            logger_.println("GPRS initialized!");
        }

        return state_.gprsConnected;
    }

    bool QuectelModem::activateGprs()
    {
        if (gprsStatus() == 1)
        {
            logger_.println("GPRS already active");
            state_.gprsConnected = true;
            return true;
        }

        if (at_.sendCommand("AT+CGATT=1", "OK"))
        {
            state_.gprsConnected = true;
            return true;
        }

        logger_.println("Failed to enable GPRS");
        state_.gprsConnected = false;
        return false;
    }

    bool QuectelModem::deactivateGprs()
    {
        if (gprsStatus() == 0)
        {
            logger_.println("GPRS already inactive");
            state_.gprsConnected = false;
            return true;
        }

        if (at_.sendCommand("AT+CGATT=0", "OK"))
        {
            gprsStatus();
            state_.gprsConnected = false;
            return true;
        }

        logger_.println("Failed to disable GPRS");
        return false;
    }

    int8_t QuectelModem::gprsStatus()
    {
        int16_t status = at_.readNumber("AT+CGATT?", "+CGATT: ", 0, 1);
        if (status >= 0)
            state_.cgattStatus = static_cast<uint16_t>(status);

        return static_cast<int8_t>(status);
    }

    bool QuectelModem::softReset()
    {
        clearConnectionState();
        // Never send CFUN into an incomplete binary transfer.
        if (!at_.synchronized())
            return false;

        // EC200U documents a maximum response time of 15 seconds for CFUN.
        if (!at_.sendCommand("AT+CFUN=1,1", "OK", 15000UL))
        {
            // A lost acknowledgement can mean the reboot has already begun.
            // Allow it to finish before the caller attempts hardware recovery.
            if (!at_.lastReplyComplete())
                delay(serialConfig_.resetWarmupMs);
            at_.invalidateSession();
            logger_.println("Software reset was not acknowledged");
            return false;
        }

        logger_.println("Waiting for GSM software reboot...");
        delay(serialConfig_.resetWarmupMs);
        at_.resetSession();
        state_.connected = waitForReady();
        if (!state_.connected)
            at_.invalidateSession();
        return state_.connected;
    }

    void QuectelModem::restart()
    {
        logger_.println("Restarting GSM");

        if (!initialize())
        {
            logger_.println("GSM not fully configured");
            logger_.print("Failure point: ");
            logger_.println(state_.initError);
            logger_.println();
        }
    }

    bool QuectelModem::hardwareReset(ResetSequence sequence)
    {
        clearConnectionState();
        if (serialConfig_.resetPin < 0)
            return false;

        const bool idleHigh = sequence == ResetSequence::HighLowHigh;
        digitalWrite(serialConfig_.resetPin, idleHigh ? HIGH : LOW);
        pinMode(serialConfig_.resetPin, OUTPUT);
        // EC200U RESET_N requires at least 100 ms asserted.
        uint32_t timingDelayMs = serialConfig_.resetPulseMs;
        if (timingDelayMs < 100)
            timingDelayMs = 100;

        if (sequence == ResetSequence::LowHighLow)
        {
            digitalWrite(serialConfig_.resetPin, LOW);
            delay(timingDelayMs);
            digitalWrite(serialConfig_.resetPin, HIGH);
            delay(timingDelayMs);
            digitalWrite(serialConfig_.resetPin, LOW);
        }
        else
        {
            digitalWrite(serialConfig_.resetPin, HIGH);
            delay(timingDelayMs);
            digitalWrite(serialConfig_.resetPin, LOW);
            delay(timingDelayMs);
            digitalWrite(serialConfig_.resetPin, HIGH);
        }

        delay(serialConfig_.resetWarmupMs);
        at_.resetSession();
        state_.connected = waitForReady();
        if (!state_.connected)
            at_.invalidateSession();
        return state_.connected;
    }

    void QuectelModem::sleep()
    {
        if (at_.sendCommand("AT+QSCLK=2", "OK"))
            logger_.println("GSM module is now in sleep mode. Will only wake up if data is sent on the serial port");
        else
            logger_.println("Failed to put GSM module in sleep mode");
    }

    void QuectelModem::troubleshoot()
    {
        if (!initialize())
            return;
        registerToNetwork();
        initializeGprs();

        failures_.httpConfigConnectFailures = 0;
        failures_.httpPostFailures = 0;
        failures_.gprsInitFailures = 0;
    }

    String QuectelModem::firmwareVersion()
    {
        String response;
        if (!at_.sendCommand("AT+GMR", "OK", response))
            return "";

        return trimmedResponseBody(response);
    }

    String QuectelModem::modelId()
    {
        String response;
        if (!at_.sendCommand("AT+GMM", "OK", response))
            return "";

        return trimmedResponseBody(response);
    }

    String QuectelModem::productInfo()
    {
        String response;
        if (!at_.sendCommand("ATI", "OK", response))
            return "";

        return trimmedResponseBody(response);
    }

    String QuectelModem::imei()
    {
        String response;
        char imeiBuffer[24] = {};

        if (!at_.sendCommand("AT+GSN=1", "OK", response))
            return "";

        if (AtClient::extractText(response.c_str(), "\"", imeiBuffer, sizeof(imeiBuffer), '"'))
            return String(imeiBuffer);

        String body = trimmedResponseBody(response);
        body.replace("+GSN:", "");
        body.replace("\"", "");
        body.trim();
        return body;
    }

    String QuectelModem::networkName()
    {
        String response;
        char networkNameBuffer[64] = {};

        if (!at_.sendCommand("AT+QSPN", "OK", response, 300))
            return "";

        if (AtClient::extractText(response.c_str(), "+QSPN: \"", networkNameBuffer, sizeof(networkNameBuffer), '"'))
        {
            state_.networkName = String(networkNameBuffer);
            return state_.networkName;
        }

        state_.networkName = "";
        return state_.networkName;
    }

    int8_t QuectelModem::signalStrength()
    {
        String response;
        char rssi[4] = {};

        if (!at_.sendCommand("AT+CSQ", "OK", response, 300))
            return 99;

        if (AtClient::extractText(response.c_str(), "+CSQ: ", rssi, sizeof(rssi), ','))
            return atoi(rssi);

        return 99;
    }

    String QuectelModem::networkBand()
    {
        String response;
        char band[64] = {};

        if (!at_.sendCommand("AT+QNWINFO", "OK", response, 300))
            return "";

        const char *start = strchr(response.c_str(), ',');
        if (start != nullptr)
            start = strchr(start + 1, ',');

        if (start == nullptr)
            return "";

        start++;
        if (*start == '"')
            start++;

        const char *end = strchr(start, '"');
        if (end == nullptr)
            return "";

        size_t length = static_cast<size_t>(end - start);
        if (length >= sizeof(band))
            return "";

        strncpy(band, start, length);
        band[length] = '\0';
        return String(band);
    }

    String QuectelModem::batteryStatus()
    {
        String response;
        char buffer[64] = {};

        if (!at_.sendCommand("AT+CBC", "OK", response, 300))
            return "";

        if (!AtClient::extractText(response.c_str(), "+CBC: ", buffer, sizeof(buffer), '\n'))
            return "";

        int chargeStatus = 0;
        int batteryLevel = 0;
        int voltage = 0;
        if (sscanf(buffer, "%d,%d,%d", &chargeStatus, &batteryLevel, &voltage) != 3)
            return "";

        const char *statusText = "Unknown";
        switch (chargeStatus)
        {
        case 0:
            statusText = "Not charging";
            break;
        case 1:
            statusText = "Charging";
            break;
        case 2:
            statusText = "Charging complete";
            break;
        default:
            break;
        }

        char formatted[64] = {};
        snprintf(formatted, sizeof(formatted), "%s %d%% %.1fv", statusText, batteryLevel, voltage / 1000.0);
        return String(formatted);
    }

    bool QuectelModem::getNetworkTime(char *time, size_t timeSize)
    {
        if (time == nullptr || timeSize == 0)
            return false;

        String response;
        char timeBuffer[32] = {};
        uint8_t retries = 0;

        at_.sendCommand("AT+CCLK?", "OK", response);
        while (!AtClient::extractText(response.c_str(), "+CCLK: \"", timeBuffer, sizeof(timeBuffer), '"') && retries < 10)
        {
            at_.sendCommand("AT+CCLK?", "OK", response);
            retries++;
            delay(1000);
        }

        String timeString = String(timeBuffer);
        if (timeString.charAt(2) == '/' && timeString.charAt(5) == '/' &&
            timeString.charAt(8) == ',' && timeString.charAt(11) == ':' && timeString.charAt(14) == ':')
        {
            strncpy(time, timeBuffer, timeSize - 1);
            time[timeSize - 1] = '\0';
            return true;
        }

        return false;
    }

    bool QuectelModem::setNetworkMode(NetworkMode mode)
    {
        char command[32] = {};
        snprintf(command, sizeof(command), "AT+QCFG=\"nwscanmode\",%d", static_cast<int>(mode));

        logger_.print("Setting network mode to: ");
        logger_.println(networkModeName(mode));

        if (!at_.sendCommand(command, "OK", 2000))
        {
            logger_.print("Failed to set network mode: ");
            logger_.println(networkModeName(mode));
            return false;
        }

        delay(1000);
        state_.currentNetwork = mode;
        return true;
    }

    void QuectelModem::cycleNetworkMode()
    {
        NetworkMode requestedMode = state_.currentNetwork;
        bool modeSet = setNetworkMode(requestedMode);
        state_.currentNetwork = nextNetworkMode(requestedMode);

        if (!modeSet)
            setNetworkMode(state_.currentNetwork);
    }

    bool QuectelModem::pingIp(const char *host, uint8_t contextId)
    {
        if (host == nullptr || host[0] == '\0')
            return false;

        char command[256] = {};
        snprintf(command, sizeof(command), "AT+QPING=%d,\"%s\",32,4", contextId, host);

        // at_.flush();
        if (!at_.sendCommand(command, "OK", 2000))
            return false;

        char urc[64] = {};
        if (!at_.waitForUrc("+QPING:", urc, sizeof(urc), 5000))
            return false;

        logger_.println(urc);
        return strstr(urc, "+QPING:") != nullptr;
    }

    bool QuectelModem::fileExists(const char *filename)
    {
        return fileSystem_.exists(filename);
    }

    bool QuectelModem::writeFile(const char *filename, const char *content)
    {
        return fileSystem_.writeText(filename, content);
    }

    bool QuectelModem::setCaCertificate(const char *certPath, uint8_t sslContextId)
    {
        if (certPath == nullptr || certPath[0] == '\0')
            return false;

        char command[96] = {};
        snprintf(command, sizeof(command), "AT+QSSLCFG=\"cacert\",%d,\"UFS:%s\"", sslContextId, certPath);
        return at_.sendCommand(command);
    }

    bool QuectelModem::resetHttpConfig()
    {
        String response;
        if (!at_.sendCommand("AT+QHTTPCFG=\"reset\"", "OK", response, 5000))
        {
            logger_.println("Failed to reset HTTP(S) config");
            logger_.println(response);
            return false;
        }

        logger_.println(response);
        return true;
    }

    bool QuectelModem::configureHttp()
    {
        String response;
        bool configured = true;

        configured &= at_.sendCommand("AT+QHTTPCFG=\"contextid\",1", "OK", response);
        logger_.println(response);
        configured &= at_.sendCommand("AT+QHTTPCFG=\"requestheader\",0", "OK", response);
        logger_.println(response);
        configured &= at_.sendCommand("AT+QHTTPCFG=\"responseheader\",1", "OK", response);
        logger_.println(response);
        configured &= at_.sendCommand("AT+QHTTPCFG=\"rspout/auto\",0", "OK", response);
        logger_.println(response);

        if (!configured)
            logger_.println("Failed to configure shared HTTP(S) settings");

        return configured;
    }

    bool QuectelModem::configureHttps()
    {
        String response;
        bool configured = true;

        configured &= at_.sendCommand("AT+QHTTPCFG=\"sslctxid\",1", "OK", response);
        logger_.println(response);
        configured &= at_.sendCommand("AT+QSSLCFG=\"sslversion\",1,3", "OK", response);
        logger_.println(response);
        configured &= at_.sendCommand("AT+QSSLCFG=\"ciphersuite\",1,0XFFFF", "OK", response);
        logger_.println(response);
        configured &= at_.sendCommand("AT+QSSLCFG=\"seclevel\",1,0", "OK", response);
        logger_.println(response);
        configured &= at_.sendCommand("AT+QSSLCFG=\"ignorelocaltime\",1,1", "OK", response);
        logger_.println(response);
        configured &= at_.sendCommand("AT+QSSLCFG=\"sni\",1,1", "OK", response);
        logger_.println(response);

        if (!configured)
            logger_.println("Failed to configure HTTPS SSL settings");

        return configured;
    }

    int QuectelModem::post(const char *url, char headers[][256], int headerCount, const char *data, size_t dataLength)
    {
        if (url == nullptr || data == nullptr)
            return 0;

        bool isHttps = strncmp(url, "https://", 8) == 0;

        if (!resetHttpConfig() || !configureHttp())
        {
            failures_.httpConfigConnectFailures++;
            return 0;
        }

        if (isHttps)
        {
            logger_.println("HTTPS URL detected; SSL context enabled");
            if (!configureHttps())
            {
                failures_.httpConfigConnectFailures++;
                return 0;
            }
        }

        char command[384] = {};
        int written = snprintf(command, sizeof(command), "AT+QHTTPCFG=\"url\",\"%s\"", url);
        if (written < 0 || written >= static_cast<int>(sizeof(command)))
        {
            logger_.println("HTTP URL config command too long");
            failures_.httpConfigConnectFailures++;
            return 0;
        }

        String response;
        if (!at_.sendCommand(command, "OK", response, 2000))
        {
            logger_.println("Failed to set HTTP(S) URL");
            logger_.println(response);
            failures_.httpConfigConnectFailures++;
            return 0;
        }
        logger_.println(response);

        for (int i = 0; i < headerCount; i++)
        {
            written = snprintf(command, sizeof(command), "AT+QHTTPCFG=\"header\",\"%s\"", headers[i]);
            if (written < 0 || written >= static_cast<int>(sizeof(command)))
            {
                logger_.println("HTTP header config command too long");
                failures_.httpConfigConnectFailures++;
                return 0;
            }

            logger_.print("Setting header: ");
            logger_.println(headers[i]);
            if (!at_.sendCommand(command, "OK"))
            {
                logger_.println("Failed to set header");
                return 0;
            }
            logger_.println("Header set successfully");
        }

        char status[4] = "000";
        char postCommand[40] = {};
        snprintf(postCommand, sizeof(postCommand), "AT+QHTTPPOST=%lu,30,60", static_cast<unsigned long>(dataLength));
        logger_.println(postCommand);

        if (!at_.sendCommand(postCommand, "CONNECT", response, 10000))
        {
            logger_.println("HTTP POST CONNECT FAIL");
            logger_.println(response);
            failures_.httpConfigConnectFailures++;
            return 0;
        }

        logger_.println("Posting gprs data..");
        readHttpPostStatus(data, status, sizeof(status));
        int responseStatus = atoi(status);
        failures_.httpPostResponseStatus = responseStatus;

        if (responseStatus >= 200 && responseStatus < 300)
        {
            logger_.print("Requested processed successfully with status: ");
            logger_.println(status);
        }
        else
        {
            logger_.print("Requested processing failed with status: ");
            logger_.println(status);
            failures_.httpPostFailures++;
        }

        return responseStatus;
    }

    bool QuectelModem::readHttpPostStatus(const char *data, char *status, size_t statusSize)
    {
        if (data == nullptr || status == nullptr || statusSize < 4)
            return false;

        strcpy(status, "000");
        at_.sendCommand(data);

        char urc[32] = {};
        if (!at_.waitForUrc("+QHTTPPOST: ", urc, sizeof(urc), 10000))
        {
            logger_.println("HTTP POST QURC not received!");
            return false;
        }

        const char *expectedReply = "+QHTTPPOST: 0,";
        if (AtClient::extractText(urc, expectedReply, status, statusSize, ',') ||
            AtClient::extractText(urc, expectedReply, status, statusSize, '\r'))
        {
            logger_.print("HTTP(S) response status code: ");
            logger_.println(status);
            return true;
        }

        logger_.println("Could not extract HTTP response status code");
        return false;
    }

    String QuectelModem::trimmedResponseBody(const String &response, const char *terminator) const
    {
        int start = response.indexOf('\n');
        int end = response.indexOf(terminator);

        if (start < 0)
            start = 0;
        else
            start++;

        if (end < start)
            end = response.length();

        String body = response.substring(start, end);
        body.trim();
        return body;
    }

} // namespace gsm
