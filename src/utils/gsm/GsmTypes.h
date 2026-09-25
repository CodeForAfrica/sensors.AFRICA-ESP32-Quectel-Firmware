#ifndef SENSORS_AFRICA_GSM_TYPES_H
#define SENSORS_AFRICA_GSM_TYPES_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

namespace gsm
{

enum class ResetSequence : uint8_t
{
    HighLowHigh,
    LowHighLow
};

enum class NetworkMode : uint8_t
{
    Auto = 0,
    G2 = 1,
    G4 = 3
};

enum class NetworkRegistrationStatus : uint8_t
{
    NotRegistered = 0,
    RegisteredHome = 1,
    Searching = 2,
    Denied = 3,
    Unknown = 4,
    RegisteredRoaming = 5
};

enum class MqttConnectionStatus : uint8_t
{
    Disconnected = 0,
    BrokerOpen = 1,
    Connected = 2
};

struct RuntimeInfo
{
    String operatorName;
    int8_t signalStrength = 99;
    String networkBand[16];
    char networkTechnology[8] = {};
    char simCcid[21] = {};
    String imei;
    String firmwareVersion;
    String modelId;
};

struct SerialConfig
{
    int8_t powerKeyPin = -1;
    int8_t resetPin = -1;
    int8_t rxPin = -1;
    int8_t txPin = -1;
    uint32_t baudRate = 115200;
    uint32_t serialWarmupMs = 4000;
    uint32_t resetWarmupMs = 30000;
    bool debugEnabled = false;
    // MCU GPIO sequence for this board's reset driver, not modem RESET_N levels.
    ResetSequence resetSequence = ResetSequence::LowHighLow;
    uint32_t resetPulseMs = 120;
};

struct ModemState
{
    char simPin[5] = {};
    bool connected = false;
    bool simAvailable = false;
    bool gprsConnected = false;
    bool simPinSet = false;
    bool simUsable = false;
    uint16_t cgattStatus = 0;
    char simCcid[21] = {};
    String initError;
    String networkName;
    NetworkMode currentNetwork = NetworkMode::Auto;
};

struct FailureCounters
{
    int httpConfigConnectFailures = 0;
    int gprsInitFailures = 0;
    int httpPostFailures = 0;
    int networkRegistrationFailures = 0;
    uint16_t httpPostResponseStatus = 0;
    int mqttInitFailures = 0;
    int mqttPublishFailures = 0;
    String mqttInitError;
    MqttConnectionStatus mqttStatus = MqttConnectionStatus::Disconnected;
};

const char *networkModeName(NetworkMode mode);
const char *registrationStatusText(int status);

} // namespace gsm

#endif // SENSORS_AFRICA_GSM_TYPES_H
