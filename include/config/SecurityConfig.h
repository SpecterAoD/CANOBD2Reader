#pragma once

#include <cstddef>
#include <cstdint>

#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #include "secrets.example.h"
#endif

namespace SecurityConfig {

constexpr bool EnableAuthentication = true;
constexpr const char* AuthenticationRealm = "CANOBD2";
constexpr const char* WebUsername = Secrets::WebUsername;
constexpr const char* WebPassword = Secrets::WebPassword;
constexpr const char* ApiToken = Secrets::ApiToken;

constexpr bool RequireOtaAuthentication = true;
constexpr bool RequireSimulationAuthentication = true;
constexpr bool RequireRestartAuthentication = true;
constexpr bool RejectOtaWhenSketchSpaceUnknown = true;
// Web-OTA cannot safely infer the PlatformIO environment from a raw ESP32 image.
// Requiring a target-specific filename catches accidental sender/display swaps
// before Update.begin() touches flash. CI produces sender.bin and display.bin.
constexpr bool RequireOtaTargetInFilename = true;
constexpr uint32_t RestartDelayMs = 300;

}
