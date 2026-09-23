#ifndef SENSORS_AFRICA_GSM_MQTT_CLIENT_H
#define SENSORS_AFRICA_GSM_MQTT_CLIENT_H

#include "AtClient.h"
#include "GsmTypes.h"

namespace gsm
{

class GsmMqttClient
{
public:
    GsmMqttClient(AtClient &atClient, Stream &logger, FailureCounters &failures);

    bool configure(uint8_t clientId = 0, uint8_t receiveMode = 0, uint8_t messageLengthEnabled = 1);
    bool open(uint8_t clientId, const char *broker, uint16_t port);
    bool connect(uint8_t clientId, const char *clientIdText, const char *username = nullptr, const char *password = nullptr);
    bool subscribe(uint8_t clientId, uint16_t messageId, const char *topic, uint8_t qos = 0);
    bool publish(uint8_t clientId, uint16_t messageId, const char *topic, const char *payload, uint8_t qos = 0, uint8_t retain = 0);
    bool unsubscribe(uint8_t clientId, uint16_t messageId, const char *topic);
    bool disconnect(uint8_t clientId);

    MqttConnectionStatus status(uint8_t clientId);
    bool isClientConnected(uint8_t clientId);
    bool isBrokerConnected(uint8_t clientId);
    bool hasBufferedMessage(uint8_t clientId);
    bool readBufferedMessage(uint8_t clientId, char *topicOut, size_t topicSize, char *payloadOut, size_t payloadSize);
    bool readBufferedMessage(uint8_t clientId, uint8_t receiveId, char *topicOut, size_t topicSize, char *payloadOut, size_t payloadSize);

private:
    AtClient &at_;
    Stream &logger_;
    FailureCounters &failures_;

    bool parseBufferedMessage(const String &fullResponse, char *topicOut, size_t topicSize, char *payloadOut, size_t payloadSize, bool logPayload);
};

} // namespace gsm

#endif // SENSORS_AFRICA_GSM_MQTT_CLIENT_H
