#pragma once

// Device-local credentials (public repository friendly)
// =====================================================
//
// Release images built by GitHub Actions contain NO credentials. Every device
// keeps its credentials in NVS (namespace "canobd2sec"):
//
//   * Provisioning: flash a locally built firmware once over USB with your
//     include/secrets.h (copy of include/secrets.example.h). On boot the
//     compiled values are validated and written to NVS.
//   * Updates: signed release images (Web/GitHub OTA) have no compiled
//     secrets and simply keep using the stored NVS values.
//   * A locally built image with valid secrets always wins and overwrites NVS,
//     so changing a password = edit secrets.h + flash once.
//   * Without valid credentials the device refuses to open its access point,
//     web server and ESP-NOW link (nothing runs with publicly known values).

#include <cstddef>
#include <cstdint>

namespace Security {

struct DeviceSecrets {
    char senderApSsid[33];
    char senderApPassword[65];
    char displayApSsid[33];
    char displayApPassword[65];
    char webUser[33];
    char webPassword[65];
    char apiToken[65];
    char stationSsid[33];      // optional default for update WiFi / hotspot
    char stationPassword[65];
    uint8_t espNowKey[16];
    uint8_t displayMac[6];
    uint8_t senderMac[6];
};

enum class SecretsSource : uint8_t {
    None,       // not provisioned: network features stay off
    Compiled,   // taken from a local include/secrets.h (and stored to NVS)
    Stored      // loaded from NVS
};

const char* sourceName(SecretsSource source);

// Writes a human readable list of problems ("; " separated) into `out`.
// Returns true when the secrets are usable for the given target.
bool validateSecrets(const DeviceSecrets& secrets, bool isSender, char* out, std::size_t outSize);

// Pure decision logic (unit tested): picks compiled secrets if valid, else
// stored secrets if valid. `storeCompiled` tells the caller to persist.
SecretsSource chooseSecrets(const DeviceSecrets* compiled,
                            const DeviceSecrets* stored,
                            bool isSender,
                            DeviceSecrets& out,
                            bool& storeCompiled);

// True for empty values and documented placeholders ("change-me...").
bool isPlaceholderText(const char* value);

// --- Runtime access (implemented for Arduino, stubbed for native tests) ---

// Loads/provisions secrets. Call once early in setup().
void begin(bool isSender);
bool provisioned();
SecretsSource source();
const DeviceSecrets& secrets();
// Problems found during begin(); empty when provisioned.
const char* problemText();

} // namespace Security
