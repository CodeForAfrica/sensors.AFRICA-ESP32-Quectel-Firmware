#ifndef SENSORS_AFRICA_QUECTEL_GNSS_H
#define SENSORS_AFRICA_QUECTEL_GNSS_H

#include "AtClient.h"

namespace gsm
{

    enum class GnssResult : uint8_t
    {
        Fix,
        NoFix,
        NotRunning,
        Unsupported,
        Error,
        InvalidResponse
    };

    struct GnssFix
    {
        char utc[16] = {};   // hhmmss.sss, UTC (not local time)
        char date[7] = {};   // ddmmyy
        double latitude = 0; // signed decimal degrees
        double longitude = 0;
        double hdop = 0;
        double altitudeMeters = 0;
        double courseDegrees = 0;
        double speedKph = 0;
        double speedKnots = 0;
        uint8_t fixType = 0; // 2 = 2D, 3 = 3D
        uint8_t satellites = 0;
    };

    // Optional Quectel GNSS service. All calls share the modem's single AT channel.
    // A successful start does not imply a fix; poll readFix from the application.
    class QuectelGnss
    {
    public:
        explicit QuectelGnss(AtClient &atClient) : at_(atClient) {}

        bool supported();
        bool start();
        bool stop();
        bool enabled(bool &enabled);
        // Clears fix on every unsuccessful read, so stale positions cannot be reused.
        GnssResult readFix(GnssFix &fix);

    private:
        AtClient &at_;
        static bool parseFix(const char *response, GnssFix &fix);
    };

} // namespace gsm

#endif
