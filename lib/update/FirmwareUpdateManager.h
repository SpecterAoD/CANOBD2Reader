#pragma once

// GitHub release based firmware updates: manifest check, channel selection,
// signed download via OtaSession. All installs are signature-checked.

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

    // Returns true while an update must not be installed (e.g. engine running).
    using InstallBlocker = bool (*)();

    static void begin(const char* targetName, const char* firmwareVersion, uint8_t protocolVersion);
    // Periodic hook: checks the manifest once per WiFi connection.
    static void handle();
    static void setChannel(UpdateChannel channel);
    static UpdateChannel channel();
    static void setInstallBlocker(InstallBlocker blocker);
    static bool checkNow();
    // Both install functions restart the device on success.
    static bool installLatest();
    static bool installVersion(const char* version, bool rollbackConfirmed = false);
    static String statusJson();
    static String versionsJson();
#if defined(ARDUINO)
    static void setLogCallback(LogCallback callback);
#endif
};

} // namespace FirmwareUpdate
