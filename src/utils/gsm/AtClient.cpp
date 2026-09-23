#include "AtClient.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace gsm
{

AtClient::AtClient(Stream &modemSerial, Stream &logger)
    : modemSerial_(modemSerial), logger_(logger)
{
}

Stream &AtClient::serial()
{
    return modemSerial_;
}

void AtClient::flush()
{
    while (modemSerial_.available())
        modemSerial_.read();
}

void AtClient::flushLoggerInput()
{
    while (logger_.available())
        logger_.read();
}

bool AtClient::sendCommand(const char *command, const char *expectedReply, unsigned long timeoutMs)
{
    replyComplete_ = false;
    if (!synchronized_ || command == nullptr || command[0] == '\0')
        return false;
    flush();
    modemSerial_.println(command);
    return waitForReply(expectedReply, timeoutMs);
}

bool AtClient::sendCommand(const char *command, const char *expectedReply, String &response, unsigned long timeoutMs)
{
    response = "";
    replyComplete_ = false;
    if (!synchronized_ || command == nullptr || command[0] == '\0')
        return false;
    flush();
    modemSerial_.println(command);
    return waitForReply(expectedReply, response, timeoutMs);
}

bool AtClient::waitForReply(const char *expectedReply, unsigned long timeoutMs)
{
    String response;
    return waitForReply(expectedReply, response, timeoutMs);
}

bool AtClient::waitForReply(const char *expectedReply, String &response, unsigned long timeoutMs)
{
    response = "";
    replyComplete_ = false;
    if (expectedReply == nullptr || expectedReply[0] == '\0')
        return false;

    const unsigned long start = millis();
    String line;
    bool overflow = false;
    while (millis() - start < timeoutMs)
    {
        while (modemSerial_.available() && millis() - start < timeoutMs)
        {
            char c = static_cast<char>(modemSerial_.read());
            if (response.length() < MaxResponseSize)
                response += c;
            else
                overflow = true;
            if (line.length() < MaxResponseSize)
                line += c;
            else
                overflow = true;

            // The MQTT input prompt is not terminated by a newline.
            if (strcmp(expectedReply, ">") == 0 && line == ">")
                return !overflow;
            if (c != '\n')
                continue;

            line.trim();
            if (line == "ERROR" || line.startsWith("+CME ERROR:") ||
                line.startsWith("+CMS ERROR:") || line == "NO CARRIER")
            {
                replyComplete_ = true;
                return false;
            }
            bool matched = false;
            if (strcmp(expectedReply, "OK") == 0)
                matched = line == "OK";
            else if (strcmp(expectedReply, "CONNECT") == 0)
                matched = line == "CONNECT" || line.startsWith("CONNECT ");
            else
                matched = !line.startsWith("AT") && line.indexOf(expectedReply) >= 0;
            if (matched)
            {
                replyComplete_ = true;
                return !overflow;
            }
            line = "";
        }
        delay(2);
    }
    return false;
}

size_t AtClient::readBytes(uint8_t *data, size_t length, unsigned long timeoutMs)
{
    if (data == nullptr && length != 0)
        return 0;
    size_t count = 0;
    const unsigned long start = millis();
    while (count < length && millis() - start < timeoutMs)
    {
        if (modemSerial_.available())
            data[count++] = static_cast<uint8_t>(modemSerial_.read());
        else
            delay(2);
    }
    return count;
}

size_t AtClient::writeBytes(const uint8_t *data, size_t length, unsigned long timeoutMs)
{
    if (data == nullptr && length != 0)
        return 0;
    size_t count = 0;
    const unsigned long start = millis();
    while (count < length && millis() - start < timeoutMs)
    {
        size_t written = modemSerial_.write(data + count, length - count);
        count += written;
        if (written == 0)
            delay(2);
    }
    return count;
}

void AtClient::resetSession()
{
    flush();
    synchronized_ = true;
}

bool AtClient::waitForUrc(const char *urcPrefix, char *response, size_t responseLen, unsigned long timeoutMs)
{
    if (response == nullptr || responseLen == 0 || urcPrefix == nullptr)
        return false;

    unsigned long start = millis();
    String line;
    response[0] = '\0';

    while (millis() - start < timeoutMs)
    {
        while (modemSerial_.available())
        {
            char c = modemSerial_.read();
            line += c;

            if (c == '\n')
            {
                if (line.startsWith(urcPrefix))
                {
                    if (line.length() >= responseLen)
                        return false;
                    strncpy(response, line.c_str(), responseLen - 1);
                    response[responseLen - 1] = '\0';
                    return true;
                }
                line = "";
            }

            if (line.length() > 256)
                line = "";
        }
        delay(2);
    }

    return false;
}

void AtClient::readRawResponse(const char *command, char *response, size_t responseLen, bool waitForTimeout, unsigned long timeoutMs)
{
    if (response == nullptr || responseLen == 0)
        return;

    response[0] = '\0';
    if (!synchronized_ || command == nullptr)
        return;

    flush();
    memset(response, '\0', responseLen);

    size_t position = 0;
    logger_.print("Received Command: ");
    logger_.println(command);
    modemSerial_.println(command);

    unsigned long start = millis();
    do
    {
        while (modemSerial_.available() && position < responseLen - 1)
        {
            response[position++] = modemSerial_.read();
        }

        if (position >= responseLen - 1)
            break;

        delay(2);
    } while ((waitForTimeout || position == 0) && (millis() - start < timeoutMs));

    logger_.println("\n-------\r\nGSM RAW RESPONSE:");
    logger_.println(response);
    logger_.println("-------");
}

int16_t AtClient::readNumber(const char *command, const char *expectedReply, uint8_t indexFrom, uint8_t length)
{
    char number[8] = {};

    if (length >= sizeof(number))
    {
        logger_.println("Max length allowed is 7");
        return -1;
    }

    String response;
    if (!sendCommand(command, "OK", response))
        return -1;

    const char *foundTarget = strstr(response.c_str(), expectedReply);
    if (foundTarget == nullptr)
        return -1;

    const char *field = foundTarget + strlen(expectedReply);
    if (strlen(field) < static_cast<size_t>(indexFrom) + length)
        return -1;
    const char *start = field + indexFrom;
    for (uint8_t i = 0; i < length; ++i)
        if (!isdigit(static_cast<unsigned char>(start[i])))
            return -1;
    strncpy(number, start, length);
    number[length] = '\0';

    return atoi(number);
}

bool AtClient::extractText(const char *input, const char *target, char *output, size_t outputSize, char until)
{
    if (input == nullptr || target == nullptr || output == nullptr || outputSize == 0)
        return false;

    const char *foundTarget = strstr(input, target);
    if (foundTarget == nullptr)
        return false;

    const char *start = foundTarget + strlen(target);
    const char *end = strchr(start, until);
    if (end == nullptr)
        return false;

    size_t length = static_cast<size_t>(end - start);
    if (length >= outputSize)
    {
        return false;
    }

    strncpy(output, start, length);
    output[length] = '\0';
    return true;
}

} // namespace gsm
