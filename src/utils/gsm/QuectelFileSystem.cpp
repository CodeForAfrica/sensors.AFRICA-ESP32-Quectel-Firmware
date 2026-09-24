#include "QuectelFileSystem.h"

#include <ctype.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

namespace gsm
{
    namespace
    {
        bool unsignedNumber(const char *&text, uint32_t &value)
        {
            while (*text == ' ')
                ++text;
            if (!isdigit(static_cast<unsigned char>(*text)))
                return false;
            uint32_t parsed = 0;
            while (isdigit(static_cast<unsigned char>(*text)))
            {
                unsigned digit = *text++ - '0';
                if (parsed > (UINT32_MAX - digit) / 10)
                    return false;
                parsed = parsed * 10 + digit;
            }
            value = parsed;
            return true;
        }

        const char *field(const String &response, const char *prefix)
        {
            const char *found = strstr(response.c_str(), prefix);
            return found == nullptr ? nullptr : found + strlen(prefix);
        }

        bool endOfLine(const char *text)
        {
            return *text == '\r' || *text == '\n' || *text == '\0';
        }
    } // namespace

    bool QuectelFileSystem::path(const char *filename, char *output, size_t capacity)
    {
        if (filename == nullptr)
            return false;
        if (strncmp(filename, "UFS:", 4) == 0)
            filename += 4;
        const size_t length = strlen(filename);
        if (length == 0 || length > 63 || capacity < length + 5 ||
            strcmp(filename, ".") == 0 || strcmp(filename, "..") == 0)
            return false;
        for (size_t i = 0; i < length; ++i)
        {
            unsigned char c = filename[i];
            if (c < 32 || c > 126 || strchr("\"\\/:*?", c) != nullptr)
                return false;
        }
        snprintf(output, capacity, "UFS:%s", filename);
        return true;
    }

    bool QuectelFileSystem::space(ModemStorageInfo &info)
    {
        info = ModemStorageInfo{};
        String response;
        if (!at_.sendCommand("AT+QFLDS=\"UFS\"", "OK", response, 2000))
            return false;
        const char *cursor = field(response, "+QFLDS:");
        ModemStorageInfo parsed;
        if (cursor == nullptr || !unsignedNumber(cursor, parsed.freeBytes) || *cursor++ != ',' ||
            !unsignedNumber(cursor, parsed.totalBytes) || !endOfLine(cursor) || parsed.freeBytes > parsed.totalBytes)
            return false;
        info = parsed;
        return true;
    }

    bool QuectelFileSystem::parseList(const String &response, ModemFileInfo *files, size_t capacity, size_t &count)
    {
        count = 0;
        const char *cursor = response.c_str();
        while ((cursor = strstr(cursor, "+QFLST: ")) != nullptr)
        {
            cursor += strlen("+QFLST: ");
            if (*cursor++ != '"')
                return false;
            const char *end = strchr(cursor, '"');
            if (end == nullptr || end[1] != ',' || end - cursor >= static_cast<int>(sizeof(ModemFileInfo::name)))
                return false;
            ModemFileInfo parsed;
            memcpy(parsed.name, cursor, end - cursor);
            cursor = end + 2;
            if (!unsignedNumber(cursor, parsed.size) || !endOfLine(cursor) || count >= capacity)
                return false;
            files[count++] = parsed;
        }
        return true;
    }

    bool QuectelFileSystem::list(ModemFileInfo *files, size_t capacity, size_t &count)
    {
        count = 0;
        if (files == nullptr && capacity != 0)
            return false;
        String response;
        return at_.sendCommand("AT+QFLST=\"UFS:*\"", "OK", response, 2000) &&
               parseList(response, files, capacity, count);
    }

    bool QuectelFileSystem::info(const char *filename, ModemFileInfo &info)
    {
        info = ModemFileInfo{};
        char normalized[68], command[96];
        if (!path(filename, normalized, sizeof(normalized)))
            return false;
        snprintf(command, sizeof(command), "AT+QFLST=\"%s\"", normalized);
        String response;
        ModemFileInfo parsed;
        size_t count = 0;
        if (!at_.sendCommand(command, "OK", response, 2000) ||
            !parseList(response, &parsed, 1, count) || count != 1)
            return false;
        char returnedPath[68];
        if (!path(parsed.name, returnedPath, sizeof(returnedPath)) || strcmp(normalized, returnedPath) != 0)
            return false;
        info = parsed;
        return true;
    }

    bool QuectelFileSystem::exists(const char *filename)
    {
        ModemFileInfo found;
        return info(filename, found);
    }

    bool QuectelFileSystem::remove(const char *filename)
    {
        char normalized[68], command[96];
        if (!path(filename, normalized, sizeof(normalized)))
            return false;
        snprintf(command, sizeof(command), "AT+QFDEL=\"%s\"", normalized);
        return at_.sendCommand(command, "OK", 5000);
    }

    bool QuectelFileSystem::open(const char *filename, unsigned mode, int &handle)
    {
        handle = -1;
        char normalized[68], command[96];
        if (!path(filename, normalized, sizeof(normalized)))
            return false;
        snprintf(command, sizeof(command), "AT+QFOPEN=\"%s\",%u", normalized, mode);
        String response;
        if (!at_.sendCommand(command, "OK", response, 2000))
        {
            if (!at_.lastReplyComplete())
                at_.invalidateSession();
            return false;
        }
        const char *cursor = field(response, "+QFOPEN:");
        uint32_t parsed = 0;
        if (cursor == nullptr || !unsignedNumber(cursor, parsed) || !endOfLine(cursor) || parsed > INT_MAX)
        {
            // A handle may have been allocated but cannot safely be identified.
            at_.invalidateSession();
            return false;
        }
        handle = static_cast<int>(parsed);
        return true;
    }

    bool QuectelFileSystem::close(int handle)
    {
        char command[32];
        snprintf(command, sizeof(command), "AT+QFCLOSE=%d", handle);
        const bool closed = at_.sendCommand(command, "OK", 2000);
        if (!closed)
            at_.invalidateSession(); // An allocated handle may still be live.
        return closed;
    }

    bool QuectelFileSystem::transferReply(const char *expected, String &response)
    {
        bool result = at_.waitForReply(expected, response, 6000);
        if (!at_.lastReplyComplete())
            at_.invalidateSession();
        return result;
    }

    bool QuectelFileSystem::writeText(const char *filename, const char *text)
    {
        return text != nullptr && write(filename, reinterpret_cast<const uint8_t *>(text), strlen(text));
    }

    bool QuectelFileSystem::write(const char *filename, const uint8_t *data, size_t length)
    {
        if ((data == nullptr && length != 0) || length > UINT32_MAX)
            return false;
        int handle;
        if (!open(filename, 1, handle))
            return false;
        bool success = true;
        size_t offset = 0;
        while (offset < length && success)
        {
            const size_t chunk = length - offset < TransferSize ? length - offset : TransferSize;
            char command[48];
            snprintf(command, sizeof(command), "AT+QFWRITE=%d,%u,5", handle, static_cast<unsigned>(chunk));
            String response;
            if (!at_.sendCommand(command, "CONNECT", response, 2000))
            {
                if (!at_.lastReplyComplete())
                    at_.invalidateSession();
                success = false;
                break;
            }
            const size_t sent = at_.writeBytes(data + offset, chunk, 1000);
            // Even on a short host write, wait for the modem's data timeout before close.
            const bool acknowledged = transferReply("OK", response);
            const char *cursor = field(response, "+QFWRITE:");
            uint32_t written = 0, total = 0;
            success = acknowledged && sent == chunk && cursor != nullptr &&
                      unsignedNumber(cursor, written) && *cursor++ == ',' &&
                      unsignedNumber(cursor, total) && endOfLine(cursor) &&
                      written == chunk && total == offset + chunk;
            offset += sent;
        }
        const bool closed = close(handle);
        return success && closed;
    }

    bool QuectelFileSystem::read(const char *filename, uint8_t *data, size_t capacity,
                                 size_t &bytesRead, uint32_t offset)
    {
        bytesRead = 0;
        if (data == nullptr && capacity != 0)
            return false;
        int handle;
        if (!open(filename, 2, handle))
            return false;
        char command[48];
        bool success = true;
        if (offset != 0)
        {
            snprintf(command, sizeof(command), "AT+QFSEEK=%d,%lu,0", handle, static_cast<unsigned long>(offset));
            success = at_.sendCommand(command, "OK", 2000);
        }
        while (success && bytesRead < capacity)
        {
            const size_t chunk = capacity - bytesRead < TransferSize ? capacity - bytesRead : TransferSize;
            snprintf(command, sizeof(command), "AT+QFREAD=%d,%u", handle, static_cast<unsigned>(chunk));
            String response;
            if (!at_.sendCommand(command, "CONNECT", response, 2000))
            {
                if (!at_.lastReplyComplete())
                    at_.invalidateSession();
                success = false;
                break;
            }
            const char *cursor = field(response, "CONNECT ");
            uint32_t actual = 0;
            if (cursor == nullptr || !unsignedNumber(cursor, actual) || !endOfLine(cursor) || actual > chunk)
            {
                at_.invalidateSession(); // Binary frame boundary is unknown.
                success = false;
                break;
            }
            const size_t received = at_.readBytes(data + bytesRead, actual, 2000);
            bytesRead += received;
            if (received != actual)
            {
                at_.invalidateSession();
                success = false;
                break;
            }
            success = transferReply("OK", response);
            if (actual < chunk)
                break;
        }
        const bool closed = close(handle);
        return success && closed;
    }
} // namespace gsm
