#ifndef SENSORS_AFRICA_AT_CLIENT_H
#define SENSORS_AFRICA_AT_CLIENT_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

namespace gsm
{

class AtClient
{
public:
    AtClient(Stream &modemSerial, Stream &logger);

    Stream &serial();
    void flush();
    void flushLoggerInput();

    bool sendCommand(const char *command, const char *expectedReply = "OK", unsigned long timeoutMs = 10000);
    bool sendCommand(const char *command, const char *expectedReply, String &response, unsigned long timeoutMs = 10000);
    bool waitForReply(const char *expectedReply, unsigned long timeoutMs);
    bool waitForReply(const char *expectedReply, String &response, unsigned long timeoutMs);
    bool waitForUrc(const char *urcPrefix, char *response, size_t responseLen, unsigned long timeoutMs);
    void readRawResponse(const char *command, char *response, size_t responseLen, bool waitForTimeout = false, unsigned long timeoutMs = 3000);
    int16_t readNumber(const char *command, const char *expectedReply, uint8_t indexFrom, uint8_t length);

    // Binary transfers use explicit lengths; embedded CR/LF, NUL and OK are data.
    size_t readBytes(uint8_t *data, size_t length, unsigned long timeoutMs);
    size_t writeBytes(const uint8_t *data, size_t length, unsigned long timeoutMs);
    bool synchronized() const { return synchronized_; }
    bool lastReplyComplete() const { return replyComplete_; }
    void invalidateSession() { synchronized_ = false; }
    // Call only after a hardware reset or independently confirmed command mode.
    void resetSession();

    static constexpr size_t MaxResponseSize = 4096;

    static bool extractText(const char *input, const char *target, char *output, size_t outputSize, char until);

private:
    Stream &modemSerial_;
    Stream &logger_;
    bool synchronized_ = true;
    bool replyComplete_ = false;
};

} // namespace gsm

#endif // SENSORS_AFRICA_AT_CLIENT_H
