#pragma once

// Template for device credentials.
//
// 1. Copy this file to include/secrets.h (ignored by Git, never commit it).
// 2. Replace EVERY value. Passwords need at least 8 characters, the API token
//    at least 16. Generate the ESP-NOW key randomly, e.g.
//      python -c "import secrets; print(', '.join(f'0x{b:02X}' for b in secrets.token_bytes(16)))"
// 3. Enter the real MAC addresses of both boards (printed on the serial
//    console at boot: "[BOOT] mac=...").
// 4. Flash sender and display ONCE over USB with this local build. The values
//    are stored in NVS; later signed OTA/GitHub updates keep them.
//
// Placeholders ("change-me...", all-zero key/MAC) are rejected at boot and
// the device keeps WiFi, web server and ESP-NOW switched off.

#include <cstdint>

namespace Secrets {

constexpr const char* SenderWebSsid = "CANOBD2_Sender";
constexpr const char* SenderWebPassword = "change-me-sender-ap";

constexpr const char* DisplayWebSsid = "CANOBD2_Display";
constexpr const char* DisplayWebPassword = "change-me-display-ap";

// Optional: phone hotspot / home WiFi used for GitHub updates (can also be
// set later in the web UI).
constexpr const char* WifiSsid = "";
constexpr const char* WifiPassword = "";

constexpr const char* WebUsername = "admin";
constexpr const char* WebPassword = "change-me-web";
constexpr const char* ApiToken = "change-me-api-token";

constexpr uint8_t EspNowAesKey[16] = {0};

constexpr uint8_t DisplayPeerMac[6] = {0};   // MAC of the LilyGO T-Display S3
constexpr uint8_t SenderAllowedMac[6] = {0}; // MAC of the ESP32 sender

}
