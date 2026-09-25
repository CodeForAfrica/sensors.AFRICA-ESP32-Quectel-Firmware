#ifndef SENSORS_AFRICA_QUECTEL_MODEM_H
#define SENSORS_AFRICA_QUECTEL_MODEM_H

#include "AtClient.h"
#include "GsmTypes.h"
#include "QuectelGnss.h"
#include "QuectelFileSystem.h"

namespace gsm
{

class QuectelModem
{
public:
    QuectelModem(AtClient &atClient, Stream &logger, const SerialConfig &serialConfig);

    ModemState &state();
    const ModemState &state() const;
    FailureCounters &failures();
    const FailureCounters &failures() const;

    // Configures the UART and attempts power-on. If AT is unavailable, initialize()
    // can still recover using RESET; the supplied UART must match AtClient's UART.
    bool beginSerial(HardwareSerial &serial);

    QuectelGnss &gnss() { return gnss_; }
    QuectelFileSystem &fileSystem() { return fileSystem_; }
    bool initialize();
    bool registerToNetwork();
    void setupSimPin();
    bool isSimCcidValid();

    bool initializeGprs();
    bool activateGprs();
    bool deactivateGprs();
    int8_t gprsStatus();

    // Success requires AT communication to return after the reset.
    bool softReset();
    void restart();
    // Pulse duration and subsequent boot wait are separate SerialConfig settings:
    // resetPulseMs (120 ms) and resetWarmupMs (30 s).
    bool hardwareReset(ResetSequence sequence);
    void sleep();
    void troubleshoot();

    String firmwareVersion();
    String modelId();
    String productInfo();
    String imei();
    String networkName();
    int8_t signalStrength();
    String networkBand();
    String batteryStatus();
    bool getNetworkTime(char *time, size_t timeSize);

    bool setNetworkMode(NetworkMode mode);
    void cycleNetworkMode();
    bool pingIp(const char *host, uint8_t contextId = 1);

    bool fileExists(const char *filename);
    bool writeFile(const char *filename, const char *content);
    bool setCaCertificate(const char *certPath, uint8_t sslContextId = 1);

    bool resetHttpConfig();
    bool configureHttp();
    bool configureHttps();
    int post(const char *url, char headers[][256], int headerCount, const char *data, size_t dataLength);

private:
    AtClient &at_;
    Stream &logger_;
    SerialConfig serialConfig_;
    QuectelGnss gnss_;
    QuectelFileSystem fileSystem_;
    ModemState state_;
    FailureCounters failures_;

    bool waitForReady(unsigned long timeoutMs = 60000);
    void clearConnectionState();
    bool configureCommands();

    bool readHttpPostStatus(const char *data, char *status, size_t statusSize);
    String trimmedResponseBody(const String &response, const char *terminator = "OK") const;
};

} // namespace gsm

#endif // SENSORS_AFRICA_QUECTEL_MODEM_H
