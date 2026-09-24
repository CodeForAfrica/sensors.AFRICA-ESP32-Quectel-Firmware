#ifndef SENSORS_AFRICA_QUECTEL_FILE_SYSTEM_H
#define SENSORS_AFRICA_QUECTEL_FILE_SYSTEM_H

#include "AtClient.h"

namespace gsm
{

    struct ModemFileInfo
    {
        char name[68] = {}; // UFS: plus up to 63 filename bytes and terminator
        uint32_t size = 0;
    };

    struct ModemStorageInfo
    {
        uint32_t freeBytes = 0;
        uint32_t totalBytes = 0;
    };

    // Root files in the modem's UFS, independent of the MCU's LittleFS/SD.
    // Accepts "name" or "UFS:name". Paths, wildcards and AT delimiters are rejected.
    class QuectelFileSystem
    {
    public:
        explicit QuectelFileSystem(AtClient &atClient) : at_(atClient) {}

        bool space(ModemStorageInfo &info);
        bool list(ModemFileInfo *files, size_t capacity, size_t &count);
        bool info(const char *filename, ModemFileInfo &info);
        bool exists(const char *filename);
        bool remove(const char *filename);
        // Binary-safe replacement, including truncation of an existing file.
        // Failure may leave a partial file; replacement is not atomic.
        bool write(const char *filename, const uint8_t *data, size_t length);
        bool writeText(const char *filename, const char *text);
        // Reads up to capacity bytes, starting at offset. EOF is a successful short read.
        // bytesRead reports bytes received even on failure; data is not NUL-terminated.
        bool read(const char *filename, uint8_t *data, size_t capacity,
                  size_t &bytesRead, uint32_t offset = 0);

    private:
        AtClient &at_;
        static constexpr size_t TransferSize = 512;

        static bool path(const char *filename, char *output, size_t capacity);
        static bool parseList(const String &response, ModemFileInfo *files, size_t capacity, size_t &count);
        bool open(const char *filename, unsigned mode, int &handle);
        bool close(int handle);
        bool transferReply(const char *expected, String &response);
    };

} // namespace gsm

#endif
