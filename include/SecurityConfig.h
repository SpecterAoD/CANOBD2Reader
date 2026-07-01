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
constexpr uint32_t RestartDelayMs = 300;

}
