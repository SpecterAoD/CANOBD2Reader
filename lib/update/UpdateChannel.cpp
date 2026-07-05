#include "UpdateChannel.h"

#include <cstring>

namespace FirmwareUpdate {

namespace {
bool equalsIgnoreCase(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) return false;
    while (*a != '\0' && *b != '\0') {
        char ca = *a++;
        char cb = *b++;
        if (ca >= 'A' && ca <= 'Z') ca = static_cast<char>(ca - 'A' + 'a');
        if (cb >= 'A' && cb <= 'Z') cb = static_cast<char>(cb - 'A' + 'a');
        if (ca != cb) return false;
    }
    return *a == '\0' && *b == '\0';
}
}

const char* channelName(UpdateChannel channel) {
    switch (channel) {
        case UpdateChannel::Stable: return "stable";
        case UpdateChannel::Beta: return "beta";
        case UpdateChannel::Development: return "development";
    }
    return "development";
}

UpdateChannel parseChannel(const char* value, UpdateChannel fallback) {
    if (equalsIgnoreCase(value, "stable")) return UpdateChannel::Stable;
    if (equalsIgnoreCase(value, "beta") || equalsIgnoreCase(value, "prerelease")) return UpdateChannel::Beta;
    if (equalsIgnoreCase(value, "development") || equalsIgnoreCase(value, "dev") ||
        equalsIgnoreCase(value, "test") || equalsIgnoreCase(value, "nightly")) {
        return UpdateChannel::Development;
    }
    return fallback;
}

bool channelAllows(UpdateChannel selected, UpdateChannel candidate) {
    if (selected == UpdateChannel::Development) return true;
    if (selected == UpdateChannel::Beta) {
        return candidate == UpdateChannel::Stable || candidate == UpdateChannel::Beta;
    }
    return candidate == UpdateChannel::Stable;
}

} // namespace FirmwareUpdate

