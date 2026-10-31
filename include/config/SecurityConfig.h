#pragma once

#include <cstddef>
#include <cstdint>

// Web/OTA security policy. Credentials themselves come from
// lib/security/DeviceSecrets (NVS), never from this header.
namespace SecurityConfig {

constexpr bool EnableAuthentication = true;
constexpr const char* AuthenticationRealm = "CANOBD2";

constexpr bool RequireOtaAuthentication = true;
constexpr bool RequireSimulationAuthentication = true;
constexpr bool RequireRestartAuthentication = true;
constexpr uint32_t RestartDelayMs = 300;

}
