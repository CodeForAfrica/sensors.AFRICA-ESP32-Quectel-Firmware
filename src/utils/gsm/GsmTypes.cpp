#include "GsmTypes.h"

namespace gsm
{

namespace
{
const char *const kRegistrationStatusText[] = {
    "Not registered to network",
    "Registered to home network",
    "Searching for network",
    "Network registration denied",
    "Network registration status unknown",
    "Registered to roaming network"};
}

const char *networkModeName(NetworkMode mode)
{
    switch (mode)
    {
    case NetworkMode::G2:
        return "2G";
    case NetworkMode::G4:
        return "4G";
    case NetworkMode::Auto:
    default:
        return "AUTO";
    }
}

const char *registrationStatusText(int status)
{
    if (status >= 0 && status < static_cast<int>(sizeof(kRegistrationStatusText) / sizeof(kRegistrationStatusText[0])))
        return kRegistrationStatusText[status];

    return "Network registration status unavailable";
}

} // namespace gsm
