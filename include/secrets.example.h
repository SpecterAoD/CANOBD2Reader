#pragma once

#include <cstdint>

// Copy this file to include/secrets.h and replace every value with device-local
// credentials before using the firmware outside a trusted test bench.
//
// include/secrets.h is intentionally ignored by Git. Keep real WiFi passwords,
// ESP-NOW keys and web credentials out of the repository.
namespace Secrets {

constexpr const char* SenderWebSsid = "CANOBD2_Sender_Setup";
constexpr const char* SenderWebPassword = "change-me-sender-ap";
constexpr const char* SenderOtaPassword = "change-me-sender-ota";

constexpr const char* DisplayWebSsid = "CANOBD2_Display_Setup";
constexpr const char* DisplayWebPassword = "change-me-display-ap";

constexpr const char* WifiSsid = "";
constexpr const char* WifiPassword = "";

constexpr const char* WebUsername = "admin";
constexpr const char* WebPassword = "change-me-web";
constexpr const char* ApiToken = "change-me-api-token";

constexpr uint8_t EspNowAesKey[16] = {
    0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE,
    0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01
};

constexpr uint8_t DisplayPeerMac[6] = { 0xF0, 0xF5, 0xBD, 0x43, 0x29, 0x20 };
constexpr uint8_t SenderAllowedMac[6] = { 0x8C, 0x4B, 0x14, 0x27, 0xEB, 0x48 };

}
