#include "QuectelGnss.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace gsm
{
    namespace
    {
        bool number(const char *text, double &value)
        {
            if (*text == '\0')
                return false;
            char *end = nullptr;
            value = strtod(text, &end);
            return end != text && *end == '\0' && isfinite(value);
        }

        bool digits(const char *text, size_t count)
        {
            for (size_t i = 0; i < count; ++i)
                if (!isdigit(static_cast<unsigned char>(text[i])))
                    return false;
            return true;
        }

        int pair(const char *text)
        {
            return (text[0] - '0') * 10 + text[1] - '0';
        }
    } // namespace

    bool QuectelGnss::supported()
    {
        return at_.sendCommand("AT+QGPS=?", "OK", 2000);
    }

    bool QuectelGnss::start()
    {
        return at_.sendCommand("AT+QGPS=1", "OK", 2000);
    }

    bool QuectelGnss::stop()
    {
        return at_.sendCommand("AT+QGPSEND", "OK", 2000);
    }

    bool QuectelGnss::enabled(bool &enabled)
    {
        enabled = false;
        String response;
        char value[4] = {};
        if (!at_.sendCommand("AT+QGPS?", "OK", response, 2000) ||
            !AtClient::extractText(response.c_str(), "+QGPS: ", value, sizeof(value), '\r'))
            return false;
        if (strcmp(value, "0") != 0 && strcmp(value, "1") != 0)
            return false;
        enabled = value[0] == '1';
        return true;
    }

    GnssResult QuectelGnss::readFix(GnssFix &fix)
    {
        fix = GnssFix{};
        String response;
        if (!at_.sendCommand("AT+QGPSLOC=2", "OK", response, 2000))
        {
            const char *error = strstr(response.c_str(), "+CME ERROR:");
            if (error != nullptr)
            {
                switch (atoi(error + strlen("+CME ERROR:")))
                {
                case 516:
                    return GnssResult::NoFix;
                case 505:
                    return GnssResult::NotRunning;
                case 502:
                    return GnssResult::Unsupported;
                }
            }
            return GnssResult::Error;
        }
        GnssFix parsed;
        if (!parseFix(response.c_str(), parsed))
            return GnssResult::InvalidResponse;
        fix = parsed;
        return GnssResult::Fix;
    }

    bool QuectelGnss::parseFix(const char *response, GnssFix &fix)
    {
        char line[256] = {};
        if (!AtClient::extractText(response, "+QGPSLOC: ", line, sizeof(line), '\r'))
            return false;

        char *fields[11] = {line};
        for (size_t i = 1; i < 11; ++i)
        {
            char *comma = strchr(fields[i - 1], ',');
            if (comma == nullptr)
                return false;
            *comma = '\0';
            fields[i] = comma + 1;
        }
        if (strchr(fields[10], ',') != nullptr)
            return false;

        const size_t utcLength = strlen(fields[0]);
        if (utcLength < 6 || utcLength >= sizeof(fix.utc) || !digits(fields[0], 6) ||
            pair(fields[0]) > 23 || pair(fields[0] + 2) > 59 || pair(fields[0] + 4) > 60)
            return false;
        if (utcLength > 6 && (utcLength < 8 || fields[0][6] != '.' || !digits(fields[0] + 7, utcLength - 7)))
            return false;
        if (strlen(fields[9]) != 6 || !digits(fields[9], 6))
            return false;
        int day = pair(fields[9]);
        int month = pair(fields[9] + 2);
        int year = 2000 + pair(fields[9] + 4);
        const int days[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        if (month < 1 || month > 12 || day < 1 || day > days[month - 1] + (month == 2 && year % 4 == 0))
            return false;

        double fixType = 0, satellites = 0;
        if (!number(fields[1], fix.latitude) || !number(fields[2], fix.longitude) ||
            !number(fields[3], fix.hdop) || !number(fields[4], fix.altitudeMeters) ||
            !number(fields[5], fixType) || !number(fields[6], fix.courseDegrees) ||
            !number(fields[7], fix.speedKph) || !number(fields[8], fix.speedKnots) ||
            !number(fields[10], satellites))
            return false;
        if (fabs(fix.latitude) > 90 || fabs(fix.longitude) > 180 || fix.hdop < 0 ||
            (fixType != 2 && fixType != 3) || satellites < 0 || satellites > 99 ||
            floor(satellites) != satellites || fix.courseDegrees < 0 || fix.courseDegrees > 360 ||
            fix.speedKph < 0 || fix.speedKnots < 0)
            return false;

        strcpy(fix.utc, fields[0]);
        strcpy(fix.date, fields[9]);
        fix.fixType = static_cast<uint8_t>(fixType);
        fix.satellites = static_cast<uint8_t>(satellites);
        return true;
    }
} // namespace gsm
