#pragma once

#include <cstddef>
#include <cstdint>

#include "config/BuildConfig.h"
#include "config/ProjectConfig.h"
#include "config/SenderConfig.h"

// Credentials (AP passwords, ESP-NOW key, peer MACs) are NOT compile-time
// constants anymore; they live per device in NVS, see lib/security/DeviceSecrets.h.

namespace NetworkConfig {

constexpr uint16_t WebServerPort = 80;
constexpr std::size_t WebConsoleMaxLines = 50;
constexpr uint8_t ManagementApMaxConnections = 2;
constexpr uint32_t ManagementApWatchdogIntervalMs = 1000;
constexpr bool RequireWebStart = SenderConfig::RequireWebStart;

constexpr bool EnableStationWifi = true;
constexpr uint32_t WifiConnectTimeoutMs = 15000;

constexpr uint8_t EspNowChannel = ProjectConfig::EspNowChannel;
// ESP-NOW and infrastructure WiFi share one ESP32 radio. If the sender/display
// joins a hotspot on a different channel, ESP-NOW telemetry can silently drop.
// Keep this enabled for normal vehicle operation; disable only for dedicated
// update/debug sessions where display telemetry interruption is acceptable.
constexpr bool DisconnectStationWifiOnEspNowChannelMismatch = true;
constexpr bool UseEspNowEncryption = true;

}
