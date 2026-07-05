#pragma once

#include <string>

#if defined(ARDUINO)
  #include <Arduino.h>
#else
  using String = std::string;
#endif

#include "UpdateChannel.h"
#include "UpdateManifest.h"

namespace FirmwareUpdate {

class FirmwareUpdateManager {
public:
#if defined(ARDUINO)
    using LogCallback = void (*)(const String& message);
#endif

    static void begin(const char* targetName, const char* firmwareVersion, uint8_t protocolVersion);
    static void handle();
    static void setChannel(UpdateChannel channel);
    static UpdateChannel channel();
    static bool checkNow();
    static bool installLatest(bool manual);
    static bool installVersion(const char* version, bool manual);
    static String statusJson();
    static String versionsJson();
#if defined(ARDUINO)
    static void setLogCallback(LogCallback callback);
#endif
};

} // namespace FirmwareUpdate

