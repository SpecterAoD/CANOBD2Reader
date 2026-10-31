#include "FirmwareUpdateManager.h"

#include "FirmwareVersion.h"
#include "GitHubUpdateClient.h"
#include "OtaSession.h"
#include "config/UpdateConfig.h"

#if defined(ARDUINO)
  #include "WifiStationManager.h"
#endif

#include <string>

namespace FirmwareUpdate {
namespace {

struct State {
    const char* target = "unknown";
    const char* installed = "V0.0.0-dev";
    uint8_t protocol = 0;
    UpdateChannel channel = UpdateConfig::DefaultChannel;
    UpdateManifest manifest;
    bool manifestLoaded = false;
    bool updateAvailable = false;
    std::string availableVersion;
    std::string lastError;
    std::string lastUpdate;
    uint32_t lastCheckMs = 0;
    bool autoCheckDoneForConnection = false;
    FirmwareUpdateManager::InstallBlocker installBlocker = nullptr;
#if defined(ARDUINO)
    FirmwareUpdateManager::LogCallback log = nullptr;
#endif
};

State state;

void logLine(const char* message) {
#if defined(ARDUINO)
    if (state.log != nullptr) state.log(String(message));
#else
    (void)message;
#endif
}

uint32_t nowMs() {
#if defined(ARDUINO)
    return millis();
#else
    return 0;
#endif
}

std::string jsonEscape(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 8);
    for (char c : value) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            default:
                if (static_cast<unsigned char>(c) >= 0x20) out.push_back(c);
        }
    }
    return out;
}

void refreshAvailability() {
    const FirmwareVersionEntry* best = findBestForwardUpdate(state.manifest, state.target, state.protocol,
                                                             state.installed, state.channel);
    state.updateAvailable = best != nullptr;
    state.availableVersion = best == nullptr ? std::string() : best->version;
}

bool fail(const std::string& message) {
    state.lastError = message;
    logLine(("[UPDATE] " + message).c_str());
    return false;
}

// Downloads, verifies and activates one manifest entry. Never returns on
// success because the device restarts into the new firmware.
bool installEntry(const FirmwareVersionEntry& entry) {
    if (UpdateConfig::BlockInstallWhileVehicleRunning && state.installBlocker != nullptr && state.installBlocker()) {
        return fail("Fahrzeug aktiv - Update erst im Stand bei Motor aus");
    }

    OtaSession session;
    if (!session.begin()) return fail(session.error());

    logLine(("[UPDATE] Download " + entry.version).c_str());
    std::string error;
    if (!GitHubUpdateClient::downloadInto(entry.url.c_str(), session, error)) {
        session.abort(error.c_str());
        return fail(error);
    }
    if (!session.finish()) return fail(session.error());

    state.lastUpdate = session.installedVersion();
    state.lastError.clear();
    logLine("[UPDATE] Signatur geprueft, Neustart in neue Firmware");
#if defined(ARDUINO)
    delay(UpdateConfig::RestartAfterUpdateMs);
    ESP.restart();
#endif
    return true;
}

} // namespace

void FirmwareUpdateManager::begin(const char* target, const char* version, uint8_t protocol) {
    state.target = target == nullptr ? "unknown" : target;
    state.installed = version == nullptr ? "V0.0.0-dev" : version;
    state.protocol = protocol;
    state.channel = UpdateConfig::DefaultChannel;
}

void FirmwareUpdateManager::handle() {
#if defined(ARDUINO)
    if (!UpdateConfig::EnableGithubUpdates || !UpdateConfig::CheckOnWifiConnect) return;
    if (!WifiNet::WifiStationManager::connected()) {
        state.autoCheckDoneForConnection = false;
        return;
    }
    const bool firstCheck = !state.autoCheckDoneForConnection;
    const bool periodicCheck = state.lastCheckMs != 0 &&
                               millis() - state.lastCheckMs > UpdateConfig::AutoCheckMinIntervalMs;
    if (!firstCheck && !periodicCheck) return;

    state.autoCheckDoneForConnection = true;
    if (checkNow() && UpdateConfig::AutoInstallUpdates && state.updateAvailable) {
        installLatest();
    }
#endif
}

void FirmwareUpdateManager::setChannel(UpdateChannel channel) {
    state.channel = channel;
    refreshAvailability();
}

UpdateChannel FirmwareUpdateManager::channel() {
    return state.channel;
}

void FirmwareUpdateManager::setInstallBlocker(InstallBlocker blocker) {
    state.installBlocker = blocker;
}

bool FirmwareUpdateManager::checkNow() {
    if (!UpdateConfig::EnableGithubUpdates) return fail("GitHub-Updates deaktiviert");
#if defined(ARDUINO)
    if (!WifiNet::WifiStationManager::connected()) return fail("WLAN offline");
#endif
    std::string body;
    std::string error;
    if (!GitHubUpdateClient::fetchText(UpdateConfig::ManifestUrl, UpdateConfig::MaxManifestBytes, body, error)) {
        return fail(error);
    }
    UpdateManifest parsed;
    if (!parseManifest(body, parsed)) return fail("Manifest unlesbar");

    state.manifest = parsed;
    state.manifestLoaded = true;
    state.lastCheckMs = nowMs();
    state.lastError.clear();
    refreshAvailability();
    logLine("[UPDATE] Manifest geladen");
    return true;
}

bool FirmwareUpdateManager::installLatest() {
    if (!state.manifestLoaded && !checkNow()) return false;
    const FirmwareVersionEntry* best = findBestForwardUpdate(state.manifest, state.target, state.protocol,
                                                             state.installed, state.channel);
    if (best == nullptr) return fail("Kein neueres Update verfuegbar");
    return installEntry(*best);
}

bool FirmwareUpdateManager::installVersion(const char* version, bool rollbackConfirmed) {
    if (!state.manifestLoaded && !checkNow()) return false;
    const FirmwareVersionEntry* entry = findVersion(state.manifest, state.target, version);
    if (entry == nullptr) return fail("Version nicht im Manifest");

    const InstallEligibility eligibility = evaluateEntry(*entry, state.target, state.protocol, state.installed,
                                                         state.channel, UpdateConfig::AllowRollback);
    if (eligibility == InstallEligibility::Rollback) {
        if (!rollbackConfirmed) return fail("Rollback muss bestaetigt werden");
    } else if (eligibility != InstallEligibility::Installable) {
        return fail(std::string("Nicht installierbar: ") + eligibilityName(eligibility));
    }
    return installEntry(*entry);
}

String FirmwareUpdateManager::statusJson() {
    std::string json = "{";
    json += "\"enabled\":";
    json += UpdateConfig::EnableGithubUpdates ? "true" : "false";
    json += ",\"autoInstall\":";
    json += UpdateConfig::AutoInstallUpdates ? "true" : "false";
    json += ",\"allowRollback\":";
    json += UpdateConfig::AllowRollback ? "true" : "false";
    json += ",\"signedUpdatesOnly\":true";
    json += ",\"channel\":\"" + std::string(channelName(state.channel)) + "\"";
    json += ",\"installed\":\"" + jsonEscape(state.installed) + "\"";
    json += ",\"target\":\"" + jsonEscape(state.target) + "\"";
    json += ",\"protocol\":" + std::to_string(state.protocol);
    json += ",\"manifestLoaded\":";
    json += state.manifestLoaded ? "true" : "false";
    json += ",\"updateAvailable\":";
    json += state.updateAvailable ? "true" : "false";
    json += ",\"availableVersion\":\"" + jsonEscape(state.availableVersion) + "\"";
    json += ",\"lastCheckMs\":" + std::to_string(state.lastCheckMs);
    json += ",\"lastUpdate\":\"" + jsonEscape(state.lastUpdate) + "\"";
    json += ",\"lastError\":\"" + jsonEscape(state.lastError) + "\"";
    json += "}";
    return String(json.c_str());
}

String FirmwareUpdateManager::versionsJson() {
    if (!state.manifestLoaded) return String("{\"versions\":[]}");
    return String(manifestVersionsJson(state.manifest, state.target, state.protocol, state.installed,
                                       state.channel, UpdateConfig::AllowRollback)
                      .c_str());
}

#if defined(ARDUINO)
void FirmwareUpdateManager::setLogCallback(LogCallback callback) {
    state.log = callback;
}
#endif

} // namespace FirmwareUpdate
