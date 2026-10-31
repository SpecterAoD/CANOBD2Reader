#include "DeviceSecrets.h"

#include <cstdio>
#include <cstring>

#if defined(ARDUINO)
  #include <Arduino.h>
  #include <Preferences.h>
#endif

// A local include/secrets.h is optional. Release builds do not have one.
#if __has_include("secrets.h") && !defined(CANOBD2_IGNORE_LOCAL_SECRETS)
  #include "secrets.h"
  #define CANOBD2_HAS_LOCAL_SECRETS 1
#else
  #define CANOBD2_HAS_LOCAL_SECRETS 0
#endif

namespace Security {
namespace {

// Example key shipped with old versions of secrets.example.h; treat as public.
constexpr uint8_t LegacyExampleKey[16] = {0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE,
                                          0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01};

void append(char* out, std::size_t outSize, const char* issue) {
    if (out == nullptr || outSize == 0) return;
    const std::size_t used = std::strlen(out);
    std::snprintf(out + used, outSize - used, "%s%s", used > 0 ? "; " : "", issue);
}

bool allZero(const uint8_t* data, std::size_t size) {
    for (std::size_t index = 0; index < size; ++index) {
        if (data[index] != 0) return false;
    }
    return true;
}

bool weakPassword(const char* value) {
    // WPA2 needs >= 8 characters; anything shorter is also too weak for web auth.
    return isPlaceholderText(value) || std::strlen(value) < 8;
}

void copyText(char* target, std::size_t size, const char* value) {
    std::snprintf(target, size, "%s", value == nullptr ? "" : value);
}

} // namespace

const char* sourceName(SecretsSource source) {
    switch (source) {
        case SecretsSource::None: return "none";
        case SecretsSource::Compiled: return "compiled";
        case SecretsSource::Stored: return "stored";
    }
    return "unknown";
}

bool isPlaceholderText(const char* value) {
    return value == nullptr || value[0] == '\0' || std::strncmp(value, "change-me", 9) == 0;
}

bool validateSecrets(const DeviceSecrets& s, bool isSender, char* out, std::size_t outSize) {
    if (out != nullptr && outSize > 0) out[0] = '\0';
    bool ok = true;
    auto issue = [&](const char* text) {
        ok = false;
        append(out, outSize, text);
    };

    if (isSender) {
        if (s.senderApSsid[0] == '\0') issue("Sender-AP-SSID fehlt");
        if (weakPassword(s.senderApPassword)) issue("Sender-AP-Passwort Platzhalter/zu kurz");
    } else {
        if (s.displayApSsid[0] == '\0') issue("Display-AP-SSID fehlt");
        if (weakPassword(s.displayApPassword)) issue("Display-AP-Passwort Platzhalter/zu kurz");
    }
    if (s.webUser[0] == '\0') issue("Web-Benutzer fehlt");
    if (weakPassword(s.webPassword)) issue("Web-Passwort Platzhalter/zu kurz");
    if (isPlaceholderText(s.apiToken) || std::strlen(s.apiToken) < 16) issue("API-Token Platzhalter/zu kurz");
    if (allZero(s.espNowKey, sizeof(s.espNowKey)) ||
        std::memcmp(s.espNowKey, LegacyExampleKey, sizeof(LegacyExampleKey)) == 0) {
        issue("ESP-NOW-Key Platzhalter");
    }
    if (allZero(isSender ? s.displayMac : s.senderMac, 6)) issue("Partner-MAC fehlt");
    return ok;
}

SecretsSource chooseSecrets(const DeviceSecrets* compiled,
                            const DeviceSecrets* stored,
                            bool isSender,
                            DeviceSecrets& out,
                            bool& storeCompiled) {
    storeCompiled = false;
    if (compiled != nullptr && validateSecrets(*compiled, isSender, nullptr, 0)) {
        out = *compiled;
        storeCompiled = stored == nullptr || std::memcmp(compiled, stored, sizeof(DeviceSecrets)) != 0;
        return SecretsSource::Compiled;
    }
    if (stored != nullptr && validateSecrets(*stored, isSender, nullptr, 0)) {
        out = *stored;
        return SecretsSource::Stored;
    }
    return SecretsSource::None;
}

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
namespace {

DeviceSecrets active{};
SecretsSource activeSource = SecretsSource::None;
char problems[256] = "nicht initialisiert";

#if CANOBD2_HAS_LOCAL_SECRETS
bool compiledSecrets(DeviceSecrets& out) {
    out = DeviceSecrets{};
    copyText(out.senderApSsid, sizeof(out.senderApSsid), Secrets::SenderWebSsid);
    copyText(out.senderApPassword, sizeof(out.senderApPassword), Secrets::SenderWebPassword);
    copyText(out.displayApSsid, sizeof(out.displayApSsid), Secrets::DisplayWebSsid);
    copyText(out.displayApPassword, sizeof(out.displayApPassword), Secrets::DisplayWebPassword);
    copyText(out.webUser, sizeof(out.webUser), Secrets::WebUsername);
    copyText(out.webPassword, sizeof(out.webPassword), Secrets::WebPassword);
    copyText(out.apiToken, sizeof(out.apiToken), Secrets::ApiToken);
    copyText(out.stationSsid, sizeof(out.stationSsid), Secrets::WifiSsid);
    copyText(out.stationPassword, sizeof(out.stationPassword), Secrets::WifiPassword);
    std::memcpy(out.espNowKey, Secrets::EspNowAesKey, sizeof(out.espNowKey));
    std::memcpy(out.displayMac, Secrets::DisplayPeerMac, sizeof(out.displayMac));
    std::memcpy(out.senderMac, Secrets::SenderAllowedMac, sizeof(out.senderMac));
    return true;
}
#endif

#if defined(ARDUINO)
constexpr const char* NvsNamespace = "canobd2sec";
constexpr const char* NvsKey = "v1";

bool loadStored(DeviceSecrets& out) {
    Preferences prefs;
    if (!prefs.begin(NvsNamespace, true)) return false;
    const bool ok = prefs.getBytesLength(NvsKey) == sizeof(DeviceSecrets) &&
                    prefs.getBytes(NvsKey, &out, sizeof(DeviceSecrets)) == sizeof(DeviceSecrets);
    prefs.end();
    return ok;
}

bool store(const DeviceSecrets& value) {
    Preferences prefs;
    if (!prefs.begin(NvsNamespace, false)) return false;
    const bool ok = prefs.putBytes(NvsKey, &value, sizeof(DeviceSecrets)) == sizeof(DeviceSecrets);
    prefs.end();
    return ok;
}
#endif

} // namespace

void begin(bool isSender) {
    DeviceSecrets compiled{};
    DeviceSecrets stored{};
    bool hasCompiled = false;
    bool hasStored = false;
#if CANOBD2_HAS_LOCAL_SECRETS
    hasCompiled = compiledSecrets(compiled);
#endif
#if defined(ARDUINO)
    hasStored = loadStored(stored);
#endif

    bool storeCompiled = false;
    activeSource = chooseSecrets(hasCompiled ? &compiled : nullptr,
                                 hasStored ? &stored : nullptr,
                                 isSender, active, storeCompiled);
    problems[0] = '\0';

    if (activeSource == SecretsSource::None) {
        // Report what is wrong with the best candidate so the user can fix it.
        if (hasCompiled) {
            validateSecrets(compiled, isSender, problems, sizeof(problems));
        } else if (hasStored) {
            validateSecrets(stored, isSender, problems, sizeof(problems));
        } else {
            copyText(problems, sizeof(problems),
                     "nicht provisioniert: einmal per USB mit lokaler include/secrets.h flashen");
        }
        active = DeviceSecrets{};
        return;
    }
#if defined(ARDUINO)
    if (storeCompiled && !store(active)) copyText(problems, sizeof(problems), "NVS-Speichern fehlgeschlagen");
#else
    (void)storeCompiled;
#endif
}

bool provisioned() { return activeSource != SecretsSource::None; }
SecretsSource source() { return activeSource; }
const DeviceSecrets& secrets() { return active; }
const char* problemText() { return problems; }

} // namespace Security
