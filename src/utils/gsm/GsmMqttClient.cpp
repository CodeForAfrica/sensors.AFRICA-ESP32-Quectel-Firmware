#include "GsmMqttClient.h"

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace gsm
{

GsmMqttClient::GsmMqttClient(AtClient &atClient, Stream &logger, FailureCounters &failures)
    : at_(atClient), logger_(logger), failures_(failures)
{
}

bool GsmMqttClient::configure(uint8_t clientId, uint8_t receiveMode, uint8_t messageLengthEnabled)
{
    char command[64] = {};
    snprintf(command, sizeof(command), "AT+QMTCFG=\"recv/mode\",%d,%d,%d", clientId, receiveMode, messageLengthEnabled);

    logger_.print("MQTT Config: ");
    logger_.println(command);

    if (!at_.sendCommand(command, "OK", 5000))
    {
        failures_.mqttInitError = "Failed to configure MQTT receiving mode";
        logger_.println(failures_.mqttInitError);
        return false;
    }

    logger_.println("MQTT configured successfully");
    return true;
}

bool GsmMqttClient::open(uint8_t clientId, const char *broker, uint16_t port)
{
    if (broker == nullptr || broker[0] == '\0')
        return false;

    char command[128] = {};
    snprintf(command, sizeof(command), "AT+QMTOPEN=%d,\"%s\",%d", clientId, broker, port);

    logger_.print("MQTT Open: ");
    logger_.println(command);

    if (!at_.sendCommand(command, "OK", 10000))
    {
        failures_.mqttInitError = "Failed to open MQTT broker connection";
        logger_.println(failures_.mqttInitError);
        failures_.mqttInitFailures++;
        return false;
    }

    char expectedUrc[24] = {};
    snprintf(expectedUrc, sizeof(expectedUrc), "+QMTOPEN: %d,0", clientId);

    char urc[32] = {};
    if (!at_.waitForUrc(expectedUrc, urc, sizeof(urc), 30000))
    {
        failures_.mqttInitError = "MQTT broker open URC not received";
        logger_.println(failures_.mqttInitError);
        failures_.mqttInitFailures++;
        return false;
    }

    failures_.mqttStatus = MqttConnectionStatus::BrokerOpen;
    return true;
}

bool GsmMqttClient::connect(uint8_t clientId, const char *clientIdText, const char *username, const char *password)
{
    if (clientIdText == nullptr || clientIdText[0] == '\0')
        return false;

    char command[256] = {};
    if (username != nullptr && password != nullptr && username[0] != '\0' && password[0] != '\0')
    {
        snprintf(command, sizeof(command), "AT+QMTCONN=%d,\"%s\",\"%s\",\"%s\"", clientId, clientIdText, username, password);
    }
    else
    {
        snprintf(command, sizeof(command), "AT+QMTCONN=%d,\"%s\"", clientId, clientIdText);
    }

    logger_.print("MQTT Connect: ");
    logger_.println(command);

    if (!at_.sendCommand(command, "OK", 10000))
    {
        failures_.mqttInitError = "Failed to send MQTT connect command";
        logger_.println(failures_.mqttInitError);
        failures_.mqttInitFailures++;
        return false;
    }

    char urc[32] = {};
    if (!at_.waitForUrc("+QMTCONN:", urc, sizeof(urc), 10000))
    {
        failures_.mqttInitError = "MQTT connect URC not received";
        logger_.println(failures_.mqttInitError);
        failures_.mqttInitFailures++;
        return false;
    }

    if (strstr(urc, "+QMTCONN:") != nullptr && strstr(urc, ",0,0") != nullptr)
    {
        failures_.mqttStatus = MqttConnectionStatus::Connected;
        failures_.mqttInitFailures = 0;
        logger_.println("MQTT client connected to broker successfully");
        return true;
    }

    failures_.mqttInitError = "Invalid MQTT connect response";
    logger_.println(failures_.mqttInitError);
    failures_.mqttInitFailures++;
    return false;
}

bool GsmMqttClient::subscribe(uint8_t clientId, uint16_t messageId, const char *topic, uint8_t qos)
{
    if (topic == nullptr || topic[0] == '\0')
        return false;

    if (messageId == 0)
    {
        logger_.println("MQTT subscribe: msg_id must be >= 1");
        return false;
    }

    if (qos > 2)
        qos = 2;

    char command[128] = {};
    snprintf(command, sizeof(command), "AT+QMTSUB=%d,%d,\"%s\",%d", clientId, messageId, topic, qos);

    logger_.print("MQTT Subscribe: ");
    logger_.println(command);

    at_.flush();
    if (!at_.sendCommand(command, "OK", 10000))
    {
        logger_.println("Failed to send MQTT subscribe command");
        return false;
    }

    char urc[32] = {};
    if (!at_.waitForUrc("+QMTSUB:", urc, sizeof(urc), 5000))
    {
        logger_.println("MQTT subscribe URC not received");
        return false;
    }

    const char *field = strchr(urc, ',');
    if (field != nullptr)
        field = strchr(field + 1, ',');

    if (field == nullptr)
    {
        logger_.println("MQTT subscribe: malformed URC");
        return false;
    }

    int result = atoi(field + 1);
    if (result != 0)
    {
        logger_.print("MQTT subscribe failed with result: ");
        logger_.println(result);
        return false;
    }

    logger_.print("Subscribed to topic: ");
    logger_.println(topic);
    return true;
}

bool GsmMqttClient::publish(uint8_t clientId, uint16_t messageId, const char *topic, const char *payload, uint8_t qos, uint8_t retain)
{
    if (topic == nullptr || payload == nullptr)
        return false;

    size_t payloadLength = strlen(payload);
    if (payloadLength > 1500)
    {
        logger_.println("MQTT payload exceeds maximum length of 1500 bytes");
        return false;
    }

    if (qos > 2)
        qos = 2;
    if (retain > 1)
        retain = 1;

    char command[256] = {};
    snprintf(command, sizeof(command), "AT+QMTPUBEX=%d,%d,%d,%d,\"%s\",%d",
             clientId, messageId, qos, retain, topic, static_cast<int>(payloadLength));

    logger_.print("MQTT Publish: ");
    logger_.println(command);

    String response;
    if (!at_.sendCommand(command, ">", response, 15000))
    {
        logger_.println("MQTT publish: no '>' data-input prompt received from modem");
        failures_.mqttPublishFailures++;
        return false;
    }

    at_.serial().write(reinterpret_cast<const uint8_t *>(payload), payloadLength);

    if (!at_.waitForReply("OK", 10000))
    {
        logger_.println("MQTT publish failed - no OK response");
        failures_.mqttPublishFailures++;
        return false;
    }

    char urc[32] = {};
    if (!at_.waitForUrc("+QMTPUBEX:", urc, sizeof(urc), 5000))
    {
        logger_.println("MQTT publish URC not received");
        failures_.mqttPublishFailures++;
        return false;
    }

    char expectedPrefix[32] = {};
    snprintf(expectedPrefix, sizeof(expectedPrefix), "+QMTPUBEX: %d,%d,", clientId, messageId);
    const char *result = strstr(urc, expectedPrefix);
    if (result != nullptr && atoi(result + strlen(expectedPrefix)) == 0)
    {
        logger_.print("Published to topic: ");
        logger_.println(topic);
        failures_.mqttPublishFailures = 0;
        return true;
    }

    logger_.println("MQTT publish failed");
    failures_.mqttPublishFailures++;
    return false;
}

bool GsmMqttClient::unsubscribe(uint8_t clientId, uint16_t messageId, const char *topic)
{
    if (topic == nullptr || topic[0] == '\0')
        return false;

    char command[128] = {};
    snprintf(command, sizeof(command), "AT+QMTUNS=%d,%d,\"%s\"", clientId, messageId, topic);

    logger_.print("MQTT Unsubscribe: ");
    logger_.println(command);

    at_.flush();
    if (!at_.sendCommand(command, "OK", 10000))
    {
        logger_.println("Failed to send MQTT unsubscribe command");
        return false;
    }

    char urc[32] = {};
    if (!at_.waitForUrc("+QMTUNS:", urc, sizeof(urc), 5000))
    {
        logger_.println("MQTT unsubscribe URC not received");
        return false;
    }

    logger_.print("Unsubscribed from topic: ");
    logger_.println(topic);
    return true;
}

bool GsmMqttClient::disconnect(uint8_t clientId)
{
    char command[32] = {};
    snprintf(command, sizeof(command), "AT+QMTDISC=%d", clientId);

    logger_.print("MQTT Disconnect: ");
    logger_.println(command);

    if (!at_.sendCommand(command, "OK", 10000))
    {
        logger_.println("Failed to send MQTT disconnect command");
        return false;
    }

    char expectedUrc[16] = {};
    snprintf(expectedUrc, sizeof(expectedUrc), "+QMTDISC: %d,0", clientId);

    char urc[32] = {};
    if (!at_.waitForUrc(expectedUrc, urc, sizeof(urc), 5000))
    {
        logger_.println("MQTT disconnect URC not received");
        return false;
    }

    failures_.mqttStatus = MqttConnectionStatus::Disconnected;
    logger_.println("MQTT client disconnected successfully");
    return true;
}

MqttConnectionStatus GsmMqttClient::status(uint8_t clientId)
{
    if (isBrokerConnected(clientId))
        return MqttConnectionStatus::Connected;

    char command[32] = {};
    snprintf(command, sizeof(command), "AT+QMTOPEN?");

    String response;
    if (!at_.sendCommand(command, "OK", response, 5000))
        return MqttConnectionStatus::Disconnected;

    char searchPattern[16] = {};
    snprintf(searchPattern, sizeof(searchPattern), "+QMTOPEN: %d,", clientId);
    if (strstr(response.c_str(), searchPattern) != nullptr)
        return MqttConnectionStatus::BrokerOpen;

    return MqttConnectionStatus::Disconnected;
}

bool GsmMqttClient::isClientConnected(uint8_t clientId)
{
    return status(clientId) == MqttConnectionStatus::Connected;
}

bool GsmMqttClient::isBrokerConnected(uint8_t clientId)
{
    String response;
    if (!at_.sendCommand("AT+QMTCONN?", "OK", response, 2000))
        return false;

    char prefix[16] = {};
    snprintf(prefix, sizeof(prefix), "+QMTCONN: %d,", clientId);

    char statusText[4] = {};
    if (!AtClient::extractText(response.c_str(), prefix, statusText, sizeof(statusText), '\r'))
        return false;

    return atoi(statusText) == 3;
}

bool GsmMqttClient::hasBufferedMessage(uint8_t clientId)
{
    char expectedUrc[16] = {};
    snprintf(expectedUrc, sizeof(expectedUrc), "+QMTRECV: %d", clientId);

    char urc[16] = {};
    if (at_.waitForUrc(expectedUrc, urc, sizeof(urc), 5000))
        return true;

    String response;
    if (!at_.sendCommand("AT+QMTRECV?", "OK", response, 1000))
        return false;

    char prefix[16] = {};
    snprintf(prefix, sizeof(prefix), "+QMTRECV: %d,", clientId);

    char statusText[32] = {};
    if (!AtClient::extractText(response.c_str(), prefix, statusText, sizeof(statusText), '\r'))
        return false;

    char *cursor = statusText;
    for (uint8_t i = 0; i < 5; i++)
    {
        if (*cursor == '1')
            return true;

        cursor = strchr(cursor, ',');
        if (cursor == nullptr)
            break;
        cursor++;
    }

    return false;
}

bool GsmMqttClient::readBufferedMessage(uint8_t clientId, uint8_t receiveId, char *topicOut, size_t topicSize, char *payloadOut, size_t payloadSize)
{
    char command[32] = {};
    snprintf(command, sizeof(command), "AT+QMTRECV=%d,%d", clientId, receiveId);

    String response;
    if (!at_.sendCommand(command, "OK", response, 5000))
    {
        logger_.println("Failed to read buffered MQTT message");
        return false;
    }

    return parseBufferedMessage(response, topicOut, topicSize, payloadOut, payloadSize, true);
}

bool GsmMqttClient::readBufferedMessage(uint8_t clientId, char *topicOut, size_t topicSize, char *payloadOut, size_t payloadSize)
{
    char command[32] = {};
    snprintf(command, sizeof(command), "AT+QMTRECV=%d", clientId);

    String response;
    if (!at_.sendCommand(command, "OK", response, 5000))
        return false;

    return parseBufferedMessage(response, topicOut, topicSize, payloadOut, payloadSize, false);
}

bool GsmMqttClient::parseBufferedMessage(const String &fullResponse, char *topicOut, size_t topicSize, char *payloadOut, size_t payloadSize, bool logPayload)
{
    if (topicOut == nullptr || payloadOut == nullptr || topicSize == 0 || payloadSize == 0)
        return false;

    int startIndex = fullResponse.indexOf("+QMTRECV:");
    if (startIndex < 0)
    {
        if (logPayload)
            logger_.println("No +QMTRECV line found in response");
        return false;
    }

    String response = fullResponse.substring(startIndex);
    int pos = response.indexOf(':');
    if (pos < 0)
        return false;

    pos++;
    while (pos < response.length() && response[pos] == ' ')
        pos++;

    while (pos < response.length() && (isdigit(response[pos]) || response[pos] == ','))
        pos++;

    while (pos < response.length() && (isdigit(response[pos]) || response[pos] == ','))
        pos++;

    if (pos >= response.length() || response[pos] != '"')
        return false;
    pos++;

    int topicEnd = response.indexOf('"', pos);
    if (topicEnd < 0)
        return false;

    String topic = response.substring(pos, topicEnd);
    if (topic.length() >= topicSize)
    {
        logger_.println("Topic too long for buffer");
        return false;
    }
    strcpy(topicOut, topic.c_str());

    pos = topicEnd + 1;
    if (pos >= response.length() || response[pos] != ',')
        return false;
    pos++;

    int payloadLength = 0;
    while (pos < response.length() && isdigit(response[pos]))
    {
        payloadLength = payloadLength * 10 + (response[pos] - '0');
        pos++;
    }

    if (pos >= response.length() || response[pos] != ',')
        return false;
    pos++;

    if (pos < response.length() && response[pos] == '"')
        pos++;

    if (payloadLength >= static_cast<int>(payloadSize))
    {
        logger_.println("Payload too large for buffer");
        return false;
    }

    strncpy(payloadOut, response.c_str() + pos, payloadLength);
    payloadOut[payloadLength] = '\0';

    logger_.print(logPayload ? "MQTT message received (buffered) -> Topic: " : "\nMQTT message received (buffered) -> Topic: ");
    logger_.print(topicOut);
    logger_.print(" | Length: ");
    logger_.print(payloadLength);
    logger_.print(" | Payload: ");
    logger_.println(payloadOut);

    return true;
}

} // namespace gsm
