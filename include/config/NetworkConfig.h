#pragma once

#include <cstddef>
#include <cstdint>

#include "config/BuildConfig.h"
#include "config/ProjectConfig.h"
#include "config/SenderConfig.h"

#if __has_include("secrets.h")
  #include "secrets.h"
#else
  #include "secrets.example.h"
#endif

namespace NetworkConfig {

constexpr const char* BluetoothName = "ESP-BT";

constexpr const char* SenderWebSsid = Secrets::SenderWebSsid;
constexpr const char* SenderWebPassword = Secrets::SenderWebPassword;
constexpr const char* SenderOtaHostname = "CAN_OBD2_Gateway";
constexpr const char* SenderOtaPassword = Secrets::SenderOtaPassword;

constexpr const char* DisplayWebSsid = Secrets::DisplayWebSsid;
constexpr const char* DisplayWebPassword = Secrets::DisplayWebPassword;
constexpr const char* DisplayOtaHostname = "CANOBD2_Display";

constexpr uint16_t WebServerPort = 80;
constexpr std::size_t WebConsoleMaxLines = 50;
constexpr bool RequireWebStart = SenderConfig::RequireWebStart;

constexpr const char* WifiSsid = Secrets::WifiSsid;
constexpr const char* WifiPassword = Secrets::WifiPassword;
constexpr bool EnableStationWifi = true;
constexpr bool WifiEnable = EnableStationWifi && WifiSsid[0] != '\0';
constexpr uint32_t WifiConnectTimeoutMs = 15000;

constexpr uint8_t EspNowChannel = ProjectConfig::EspNowChannel;
constexpr bool UseEspNowEncryption = true;
constexpr const uint8_t* EspNowAesKey = Secrets::EspNowAesKey;
constexpr const uint8_t* DisplayPeerMac = Secrets::DisplayPeerMac;
constexpr const uint8_t* SenderAllowedMac = Secrets::SenderAllowedMac;

constexpr bool BluetoothEnabled = BuildConfig::BluetoothEnabled;

}
