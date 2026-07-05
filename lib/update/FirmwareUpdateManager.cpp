#include "FirmwareUpdateManager.h"

#include "GitHubUpdateClient.h"
#include "config/UpdateConfig.h"
#include "WifiStationManager.h"

#if defined(ARDUINO)
  #include <HTTPClient.h>
  #include <Update.h>
  #include <WiFiClientSecure.h>
  #include <mbedtls/sha256.h>
  #include <esp_system.h>
#endif

#include <cstdio>
#include <cstring>

namespace FirmwareUpdate {
namespace {

const char* targetName = "unknown";
const char* firmwareVersion = "V0.0.0-dev";
uint8_t protocolVersion = 0;
UpdateChannel selectedChannel = UpdateConfig::DefaultChannel;
UpdateManifest manifest;
bool manifestLoaded = false;
bool updateAvailable = false;
std::string availableVersion;
std::string lastError;
std::string lastUpdate;
uint32_t lastCheckMs = 0;
uint32_t lastInstallMs = 0;
bool autoCheckDoneForConnection = false;

#if defined(ARDUINO)
FirmwareUpdateManager::LogCallback logCallback = nullptr;

void logLine(const String& message) {
    if (logCallback != nullptr) logCallback(message);
}

String jsonEscapeArduino(const std::string& value) {
    String out;
    for (char c : value) {
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else {
            out += c;
        }
    }
    return out;
}

bool sha256Equals(const uint8_t* digest, const std::string& expected) {
    if (expected.size() != 64) return false;
    char hex[65];
    for (size_t i = 0; i < 32; ++i) {
        std::snprintf(hex + (i * 2), 3, "%02x", digest[i]);
    }
    hex[64] = '\0';
    return expected == hex;
}

bool installEntry(const FirmwareVersionEntry& entry, bool manual) {
    const auto eligibility = evaluateEntry(entry,
                                           targetName,
                                           protocolVersion,
                                           firmwareVersion,
                                           selectedChannel,
                                           UpdateConfig::AllowRollback);
    if (eligibility == InstallEligibility::Rollback && (!manual || !UpdateConfig::AllowRollback)) {
        lastError = "Rollback requires manual confirmation";
        return false;
    }
    if (eligibility != InstallEligibility::Installable && eligibility != InstallEligibility::Rollback) {
        lastError = std::string("Entry not installable: ") + eligibilityName(eligibility);
        return false;
    }
    if (ESP.getFreeSketchSpace() == 0) {
        lastError = "Free OTA sketch space unknown";
        return false;
    }

    WiFiClientSecure client;
    if (UpdateConfig::RequireTlsCertificateValidation) {
        client.setCACert(UpdateConfig::GitHubRootCa);
    } else if (UpdateConfig::AllowInsecureTlsForDevelopment) {
        client.setInsecure();
    } else {
        lastError = "TLS validation disabled by config";
        return false;
    }

    HTTPClient http;
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setTimeout(UpdateConfig::DownloadTimeoutMs);
    if (!http.begin(client, entry.url.c_str())) {
        lastError = "Firmware HTTP begin failed";
        return false;
    }
    const int code = http.GET();
    if (code != HTTP_CODE_OK) {
        lastError = "Firmware HTTP status " + std::to_string(code);
        http.end();
        return false;
    }

    const int contentLength = http.getSize();
    if (contentLength > 0 && static_cast<size_t>(contentLength) > ESP.getFreeSketchSpace()) {
        lastError = "Firmware too large for OTA partition";
        http.end();
        return false;
    }
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        lastError = "Update.begin failed";
        http.end();
        return false;
    }

    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);
    mbedtls_sha256_starts_ret(&sha, 0);

    WiFiClient* stream = http.getStreamPtr();
    uint8_t buffer[1024];
    size_t written = 0;
    uint32_t lastDataMs = millis();
    logLine("[UPDATE] Download gestartet");
    while (http.connected() && (contentLength < 0 || written < static_cast<size_t>(contentLength))) {
        const size_t available = stream->available();
        if (available > 0) {
            const int read = stream->readBytes(buffer, available > sizeof(buffer) ? sizeof(buffer) : available);
            if (read > 0) {
                lastDataMs = millis();
                mbedtls_sha256_update_ret(&sha, buffer, static_cast<size_t>(read));
                if (Update.write(buffer, static_cast<size_t>(read)) != static_cast<size_t>(read)) {
                    lastError = "Update.write failed";
                    Update.end();
                    http.end();
                    mbedtls_sha256_free(&sha);
                    return false;
                }
                written += static_cast<size_t>(read);
            }
        } else {
            if (millis() - lastDataMs > UpdateConfig::DownloadTimeoutMs) {
                lastError = "Firmware download timeout";
                Update.end();
                http.end();
                mbedtls_sha256_free(&sha);
                return false;
            }
            delay(10);
            yield();
        }
    }

    uint8_t digest[32];
    mbedtls_sha256_finish_ret(&sha, digest);
    mbedtls_sha256_free(&sha);
    http.end();

    if (!sha256Equals(digest, entry.sha256)) {
        lastError = "SHA256 mismatch";
        Update.end();
        return false;
    }
    logLine("[UPDATE] SHA256 OK");

    if (!Update.end(true)) {
        lastError = "Update.end failed";
        return false;
    }

    lastInstallMs = millis();
    lastUpdate = entry.version;
    lastError.clear();
    logLine("[UPDATE] OTA erfolgreich");
    delay(UpdateConfig::RestartAfterUpdateMs);
    ESP.restart();
    return true;
}
#else
String jsonEscapeArduino(const std::string& value) {
    return value;
}
#endif

String numberString(unsigned long value) {
#if defined(ARDUINO)
    return String(value);
#else
    return std::to_string(value);
#endif
}

void updateAvailableState() {
    const FirmwareVersionEntry* best = findBestForwardUpdate(manifest,
                                                             targetName,
                                                             protocolVersion,
                                                             firmwareVersion,
                                                             selectedChannel);
    updateAvailable = best != nullptr;
    availableVersion = best == nullptr ? std::string() : best->version;
}

} // namespace

void FirmwareUpdateManager::begin(const char* target, const char* version, uint8_t protocol) {
    targetName = target == nullptr ? "unknown" : target;
    firmwareVersion = version == nullptr ? "V0.0.0-dev" : version;
    protocolVersion = protocol;
    selectedChannel = UpdateConfig::DefaultChannel;
}

void FirmwareUpdateManager::handle() {
#if defined(ARDUINO)
    if (!UpdateConfig::EnableGithubUpdates || !UpdateConfig::EnableAutoUpdate) return;
    if (!Network::WifiStationManager::connected()) {
        autoCheckDoneForConnection = false;
        return;
    }
    if (UpdateConfig::CheckOnWifiConnect && !autoCheckDoneForConnection) {
        autoCheckDoneForConnection = true;
        checkNow();
        if (UpdateConfig::AutoInstallUpdates) {
            installLatest(false);
        }
    }
    if (lastCheckMs != 0 && millis() - lastCheckMs > UpdateConfig::AutoCheckMinIntervalMs) {
        checkNow();
    }
#endif
}

void FirmwareUpdateManager::setChannel(UpdateChannel channelValue) {
    selectedChannel = channelValue;
    updateAvailableState();
}

UpdateChannel FirmwareUpdateManager::channel() {
    return selectedChannel;
}

bool FirmwareUpdateManager::checkNow() {
    if (!UpdateConfig::EnableGithubUpdates) {
        lastError = "GitHub updates disabled";
        return false;
    }
#if defined(ARDUINO)
    if (!Network::WifiStationManager::connected()) {
        lastError = "WLAN offline";
        return false;
    }
#endif
    std::string body;
    std::string error;
    if (!GitHubUpdateClient::fetchText(UpdateConfig::ManifestUrl, body, error)) {
        lastError = error;
        return false;
    }
    if (!parseManifest(body, manifest)) {
        lastError = "Manifest parse failed";
        manifestLoaded = false;
        return false;
    }
    manifestLoaded = true;
#if defined(ARDUINO)
    lastCheckMs = millis();
    logLine("[UPDATE] Manifest geladen");
#endif
    lastError.clear();
    updateAvailableState();
    return true;
}

bool FirmwareUpdateManager::installLatest(bool manual) {
    if (!manifestLoaded && !checkNow()) return false;
    const FirmwareVersionEntry* best = findBestForwardUpdate(manifest,
                                                             targetName,
                                                             protocolVersion,
                                                             firmwareVersion,
                                                             selectedChannel);
    if (best == nullptr) {
        lastError = "No forward update available";
        return false;
    }
#if defined(ARDUINO)
    return installEntry(*best, manual);
#else
    return true;
#endif
}

bool FirmwareUpdateManager::installVersion(const char* version, bool manual, bool rollbackConfirmed) {
    if (!manifestLoaded && !checkNow()) return false;
    const FirmwareVersionEntry* entry = findVersion(manifest, targetName, version);
    if (entry == nullptr) {
        lastError = "Requested version not found";
        return false;
    }
    const auto eligibility = evaluateEntry(*entry,
                                           targetName,
                                           protocolVersion,
                                           firmwareVersion,
                                           selectedChannel,
                                           UpdateConfig::AllowRollback);
    if (eligibility != InstallEligibility::Installable &&
        eligibility != InstallEligibility::Rollback) {
        lastError = std::string("Entry not installable: ") + eligibilityName(eligibility);
        return false;
    }
    if (eligibility == InstallEligibility::Rollback &&
        (!manual || !UpdateConfig::AllowRollback)) {
        lastError = "Rollback requires manual confirmation";
        return false;
    }
    if (eligibility == InstallEligibility::Rollback &&
        UpdateConfig::RequireConfirmationForRollback &&
        !rollbackConfirmed) {
        lastError = "Rollback requires explicit confirmation";
        return false;
    }
#if defined(ARDUINO)
    return installEntry(*entry, manual);
#else
    return true;
#endif
}

String FirmwareUpdateManager::statusJson() {
    String json = "{";
    json += "\"enabled\":";
    json += UpdateConfig::EnableGithubUpdates ? "true" : "false";
    json += ",\"autoUpdate\":";
    json += UpdateConfig::EnableAutoUpdate ? "true" : "false";
    json += ",\"autoInstall\":";
    json += UpdateConfig::AutoInstallUpdates ? "true" : "false";
    json += ",\"allowRollback\":";
    json += UpdateConfig::AllowRollback ? "true" : "false";
    json += ",\"requireRollbackConfirmation\":";
    json += UpdateConfig::RequireConfirmationForRollback ? "true" : "false";
    json += ",\"channel\":\"";
    json += channelName(selectedChannel);
    json += "\",\"installed\":\"";
    json += firmwareVersion;
    json += "\",\"target\":\"";
    json += targetName;
    json += "\",\"protocol\":";
    json += numberString(static_cast<unsigned long>(protocolVersion));
    json += ",\"manifestLoaded\":";
    json += manifestLoaded ? "true" : "false";
    json += ",\"updateAvailable\":";
    json += updateAvailable ? "true" : "false";
    json += ",\"availableVersion\":\"";
    json += jsonEscapeArduino(availableVersion);
    json += "\",\"lastCheckMs\":";
    json += numberString(static_cast<unsigned long>(lastCheckMs));
    json += ",\"lastInstallMs\":";
    json += numberString(static_cast<unsigned long>(lastInstallMs));
    json += ",\"lastUpdate\":\"";
    json += jsonEscapeArduino(lastUpdate);
    json += "\",\"lastError\":\"";
    json += jsonEscapeArduino(lastError);
    json += "\"}";
    return json;
}

String FirmwareUpdateManager::versionsJson() {
    if (!manifestLoaded) {
        return "{\"versions\":[]}";
    }
    return String(manifestVersionsJson(manifest,
                                       targetName,
                                       protocolVersion,
                                       firmwareVersion,
                                       selectedChannel,
                                       UpdateConfig::AllowRollback).c_str());
}

#if defined(ARDUINO)
void FirmwareUpdateManager::setLogCallback(LogCallback callback) {
    logCallback = callback;
}
#endif

} // namespace FirmwareUpdate
