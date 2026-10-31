#if defined(ARDUINO)

#include "ManagementServer.h"

#include <WiFi.h>

#include "AuthHelpers.h"
#include "common_config.h"
#include "DeviceSecrets.h"
#include "DiagnosticLog.h"
#include "FirmwareMetadata.h"
#include "OtaSession.h"
#include "RuntimeSimulation.h"
#include "SimulationTypes.h"
#include "WebAssets.h"
#include "WifiStationManager.h"
#include "config/FirmwareSigningKeys.h"
#include "config/LoggingConfig.h"
#include "config/NetworkConfig.h"
#include "config/ProjectConfig.h"
#include "config/SecurityConfig.h"

namespace Management {
namespace {

WebServer webServer(NetworkConfig::WebServerPort);
Config config{};
bool serverRunning = false;

FirmwareUpdate::OtaSession upload;
bool uploadSucceeded = false;
bool uploadInProgress = false;
String uploadStatus = "Bereit";

uint32_t lastApCheckMs = 0;
uint32_t apRestarts = 0;

void log(const String& message) {
    if (config.log != nullptr) config.log(message);
}

bool authorized(bool protect = true) {
    return WebSecurity::requireAuthentication(webServer, protect);
}

void sendJson(int code, const String& json) {
    webServer.send(code, "application/json", json);
}

bool startAccessPoint(const char* reason) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    // The AP must stay on the ESP-NOW channel, otherwise telemetry stops.
    const bool ok = WiFi.softAP(config.apSsid, config.apPassword, NetworkConfig::EspNowChannel, false,
                                NetworkConfig::ManagementApMaxConnections);
    log(String("[WEB] Access Point ") + (ok ? "aktiv " + WiFi.softAPIP().toString() : String("Fehler")) +
        " (" + reason + ")");
    return ok;
}

void keepAccessPointAlive() {
    const uint32_t now = millis();
    if (now - lastApCheckMs < NetworkConfig::ManagementApWatchdogIntervalMs) return;
    lastApCheckMs = now;
    const wifi_mode_t mode = WiFi.getMode();
    if ((mode == WIFI_AP || mode == WIFI_AP_STA) && WiFi.softAPIP() != IPAddress(0, 0, 0, 0)) return;
    ++apRestarts;
    startAccessPoint("watchdog");
}

// ---- Signed firmware upload --------------------------------------------------

void handleUploadChunk() {
    if (!authorized(SecurityConfig::RequireOtaAuthentication)) return;
    HTTPUpload& part = webServer.upload();
    switch (part.status) {
        case UPLOAD_FILE_START:
            uploadSucceeded = false;
            uploadInProgress = true;
            if (config.installBlocker != nullptr && config.installBlocker()) {
                uploadStatus = "Abgelehnt: Fahrzeug aktiv";
                uploadInProgress = false;
                break;
            }
            uploadStatus = upload.begin() ? "Upload laeuft: " + part.filename
                                          : String("Fehler: ") + upload.error().c_str();
            log("[OTA] " + uploadStatus);
            break;
        case UPLOAD_FILE_WRITE:
            if (upload.active() && !upload.write(part.buf, part.currentSize)) {
                uploadStatus = String("Fehler: ") + upload.error().c_str();
                log("[OTA] " + uploadStatus);
            }
            break;
        case UPLOAD_FILE_END:
            if (upload.active()) {
                uploadSucceeded = upload.finish();
                uploadStatus = uploadSucceeded
                                   ? String("Signatur OK, installiert: ") + upload.installedVersion().c_str()
                                   : String("Abgelehnt: ") + upload.error().c_str();
                log("[OTA] " + uploadStatus);
            }
            uploadInProgress = false;
            break;
        case UPLOAD_FILE_ABORTED:
            upload.abort("Upload abgebrochen");
            uploadStatus = "Upload abgebrochen";
            uploadInProgress = false;
            log("[OTA] " + uploadStatus);
            break;
    }
}

void handleUploadFinished() {
    if (!authorized(SecurityConfig::RequireOtaAuthentication)) return;
    // Only a fully verified and activated image counts as success.
    if (!uploadSucceeded) {
        webServer.send(400, "text/plain; charset=utf-8", uploadStatus);
        return;
    }
    webServer.send(200, "text/plain; charset=utf-8", "Update erfolgreich, Neustart ...");
    DiagnosticLog::appendf("[OTA] Web-Update installiert, Neustart");
    delay(500);
    ESP.restart();
}

// ---- GitHub updates / WiFi ---------------------------------------------------

void handleGithubPage() {
    if (!authorized()) return;
    String page(WebAssets::githubUpdatePage());
    page.replace("%%COMMON_JS%%", WebAssets::commonScript());
    page.replace("%%DEVICE%%", config.deviceName);
    webServer.send(200, "text/html; charset=utf-8", page);
}

void handleUpdateStatus() {
    if (!authorized()) return;
    sendJson(200, FirmwareUpdate::FirmwareUpdateManager::statusJson());
}

void handleUpdateVersions() {
    if (!authorized()) return;
    sendJson(200, FirmwareUpdate::FirmwareUpdateManager::versionsJson());
}

void handleUpdateCheck() {
    if (!authorized()) return;
    const bool ok = FirmwareUpdate::FirmwareUpdateManager::checkNow();
    sendJson(ok ? 200 : 502, FirmwareUpdate::FirmwareUpdateManager::statusJson());
}

void handleUpdateInstall() {
    if (!authorized(SecurityConfig::RequireOtaAuthentication)) return;
    const String version = webServer.arg("version");
    const bool rollbackConfirmed = webServer.arg("confirm") == "rollback";
    // On success the device restarts inside the call, so only failures return.
    const bool ok = version.length() > 0
                        ? FirmwareUpdate::FirmwareUpdateManager::installVersion(version.c_str(), rollbackConfirmed)
                        : FirmwareUpdate::FirmwareUpdateManager::installLatest();
    sendJson(ok ? 200 : 409, FirmwareUpdate::FirmwareUpdateManager::statusJson());
}

void handleUpdateChannel() {
    if (!authorized()) return;
    FirmwareUpdate::FirmwareUpdateManager::setChannel(FirmwareUpdate::parseChannel(webServer.arg("channel").c_str()));
    sendJson(200, FirmwareUpdate::FirmwareUpdateManager::statusJson());
}

void handleWifiStatus() {
    if (!authorized()) return;
    sendJson(200, WifiNet::WifiStationManager::statusJson());
}

void handleWifiConfigure() {
    if (!authorized()) return;
    const bool ok = WifiNet::WifiStationManager::configure(webServer.arg("ssid"), webServer.arg("password"));
    sendJson(ok ? 200 : 400, ok ? "{\"saved\":true}" : "{\"saved\":false}");
}

void handleWifiConnect() {
    if (!authorized()) return;
    const bool ok = WifiNet::WifiStationManager::connect();
    log(ok ? "[WIFI] Verbindung gestartet" : "[WIFI] Verbindung fehlgeschlagen");
    sendJson(ok ? 200 : 500, WifiNet::WifiStationManager::statusJson());
}

void handleWifiForget() {
    if (!authorized()) return;
    const bool ok = WifiNet::WifiStationManager::forget();
    sendJson(ok ? 200 : 500, WifiNet::WifiStationManager::statusJson());
}

// ---- Logs / restart ----------------------------------------------------------

String report() {
    return config.diagnosticReport != nullptr ? config.diagnosticReport() : DiagnosticLog::readAll();
}

void handleLogFile() {
    if (!authorized()) return;
    webServer.send(200, "text/plain; charset=utf-8", report());
}

void handleLogDownload() {
    if (!authorized()) return;
    const String filename = String("CANOBD2_") + ProjectConfig::TargetName + "_" + ProjectConfig::FirmwareVersion +
                            "_diagnostic.log";
    webServer.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    webServer.send(200, "text/plain; charset=utf-8", report());
}

void handleLogClear() {
    if (!authorized()) return;
    const bool ok = DiagnosticLog::clear();
    log(String("[WEB] Diagnose-Log ") + (ok ? "geloescht" : "konnte nicht geloescht werden"));
    sendJson(ok ? 200 : 500, ok ? "{\"cleared\":true}" : "{\"cleared\":false}");
}

void handleRestart() {
    if (!authorized(SecurityConfig::RequireRestartAuthentication)) return;
    log("[WEB] Neustart angefordert");
    sendRestartAndRestart();
}

// ---- Simulation --------------------------------------------------------------

String simulationJson() {
    String json = "{\"devTools\":";
    json += CANOBD2_DEV_TOOLS ? "true" : "false";
    json += ",\"simulation\":";
    json += Simulation::RuntimeSimulation::enabled() ? "true" : "false";
    json += ",\"scenario\":\"";
    json += jsonEscape(Simulation::RuntimeSimulation::scenarioName());
    json += "\"}";
    return json;
}

void handleSimulationStatus() {
    if (!authorized(SecurityConfig::RequireSimulationAuthentication)) return;
    sendJson(200, simulationJson());
}

#if CANOBD2_DEV_TOOLS
void setSimulation(bool enabled) {
    Simulation::RuntimeSimulation::setEnabled(enabled);
    log(String("[SIM] ") + (enabled ? "eingeschaltet" : "ausgeschaltet"));
}

void handleSimulationOn() {
    if (!authorized(SecurityConfig::RequireSimulationAuthentication)) return;
    setSimulation(true);
    sendJson(200, simulationJson());
}

void handleSimulationOff() {
    if (!authorized(SecurityConfig::RequireSimulationAuthentication)) return;
    setSimulation(false);
    sendJson(200, simulationJson());
}

void handleSimulationToggle() {
    if (!authorized(SecurityConfig::RequireSimulationAuthentication)) return;
    setSimulation(!Simulation::RuntimeSimulation::enabled());
    sendJson(200, simulationJson());
}

void handleSimulationScenario() {
    if (!authorized(SecurityConfig::RequireSimulationAuthentication)) return;
    String raw = webServer.arg("scenario");
    raw.trim();
    Simulation::Scenario scenario;
    if (!Simulation::parseScenario(raw.c_str(), scenario)) {
        sendJson(400, "{\"error\":\"invalid scenario\"}");
        return;
    }
    Simulation::RuntimeSimulation::setScenario(scenario);
    log(String("[SIM] Szenario ") + Simulation::RuntimeSimulation::scenarioName());
    sendJson(200, simulationJson());
}

#endif

void registerRoutes() {
    webServer.on("/update", HTTP_POST, handleUploadFinished, handleUploadChunk);
    webServer.on("/github-update", HTTP_GET, handleGithubPage);
    webServer.on("/api/update/status", HTTP_GET, handleUpdateStatus);
    webServer.on("/api/update/versions", HTTP_GET, handleUpdateVersions);
    webServer.on("/api/update/check", HTTP_POST, handleUpdateCheck);
    webServer.on("/api/update/install", HTTP_POST, handleUpdateInstall);
    webServer.on("/api/update/channel", HTTP_POST, handleUpdateChannel);
    webServer.on("/api/wifi/status", HTTP_GET, handleWifiStatus);
    webServer.on("/api/wifi/configure", HTTP_POST, handleWifiConfigure);
    webServer.on("/api/wifi/connect", HTTP_POST, handleWifiConnect);
    webServer.on("/api/wifi/forget", HTTP_POST, handleWifiForget);
    webServer.on("/log/file", HTTP_GET, handleLogFile);
    webServer.on("/log/download", HTTP_GET, handleLogDownload);
    webServer.on("/api/log/clear", HTTP_POST, handleLogClear);
    webServer.on("/api/restart", HTTP_POST, handleRestart);
    // Simulation status is always readable; switching needs a dev build.
    webServer.on("/api/simulation", HTTP_GET, handleSimulationStatus);
#if CANOBD2_DEV_TOOLS
    webServer.on("/api/simulation/on", HTTP_POST, handleSimulationOn);
    webServer.on("/api/simulation/off", HTTP_POST, handleSimulationOff);
    webServer.on("/api/simulation/toggle", HTTP_POST, handleSimulationToggle);
    webServer.on("/api/simulation/scenario", HTTP_POST, handleSimulationScenario);
#endif
    webServer.onNotFound([]() { webServer.send(404, "text/plain", "Not found"); });
}

void updateLog(const String& message) {
    log(message);
}

} // namespace

bool begin(const Config& value) {
    config = value;
    if (!Security::provisioned()) {
        log(String("[WEB] Management deaktiviert: ") + Security::problemText());
        return false;
    }

    startAccessPoint("boot");
    WifiNet::WifiStationManager::begin();
    FirmwareUpdate::FirmwareUpdateManager::begin(ProjectConfig::TargetName, ProjectConfig::FirmwareVersion,
                                                 ProjectConfig::ProtocolVersion);
    FirmwareUpdate::FirmwareUpdateManager::setLogCallback(updateLog);
    FirmwareUpdate::FirmwareUpdateManager::setInstallBlocker(config.installBlocker);

    WebSecurity::collectAuthHeaders(webServer);
    registerRoutes();
    webServer.begin();
    serverRunning = true;
    return true;
}

void handle() {
    if (!serverRunning) return;
    keepAccessPointAlive();
    WifiNet::WifiStationManager::handle();
    webServer.handleClient();
    FirmwareUpdate::FirmwareUpdateManager::handle();
}

bool running() { return serverRunning; }
WebServer& server() { return webServer; }
bool otaInProgress() { return uploadInProgress; }
const String& otaStatus() { return uploadStatus; }

String jsonEscape(const String& value) {
    String out;
    out.reserve(value.length() + 8);
    for (unsigned int index = 0; index < value.length(); ++index) {
        const char c = value[index];
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) >= 0x20) out += c;
        }
    }
    return out;
}

void appendCommonStatusJson(String& json) {
    auto field = [&json](const char* key, const String& value) {
        json += "\"";
        json += key;
        json += "\":\"";
        json += jsonEscape(value);
        json += "\",";
    };
    auto number = [&json](const char* key, unsigned long value) {
        json += "\"";
        json += key;
        json += "\":";
        json += String(value);
        json += ",";
    };
    auto flag = [&json](const char* key, bool value) {
        json += "\"";
        json += key;
        json += "\":";
        json += value ? "true," : "false,";
    };

    field("firmware", ProjectConfig::FirmwareVersion);
    field("target", ProjectConfig::TargetName);
    number("protocol", ProjectConfig::ProtocolVersion);
    field("buildTime", String(__DATE__) + " " + __TIME__);
    // Also keeps the metadata record linked into the image: OTA validation of
    // the *next* firmware relies on every image carrying it.
    field("metadata", FirmwareMetadata::text());
    field("otaStatus", uploadStatus);
    flag("otaInProgress", uploadInProgress);
    flag("signedUpdatesOnly", true);
    number("trustedKeys", FirmwareSigningKeys::TrustedCount);
    flag("securityReady", Security::provisioned());
    field("securityWarning", Security::provisioned() ? String() : String(Security::problemText()));
    field("secretsSource", Security::sourceName(Security::source()));
    number("freeSketchSpace", ESP.getFreeSketchSpace());
    number("sketchSize", ESP.getSketchSize());
    number("flashSize", ESP.getFlashChipSize());
    number("freeHeap", ESP.getFreeHeap());
    flag("diagnosticLogMounted", DiagnosticLog::mounted());
    number("diagnosticLogSize", DiagnosticLog::size());
    number("diagnosticLogMaxSize", LoggingConfig::DiagnosticLogMaxBytes);
    field("ip", WiFi.softAPIP().toString());
    number("apClients", WiFi.softAPgetStationNum());
    number("apRestarts", apRestarts);
    flag("devTools", CANOBD2_DEV_TOOLS);
    number("uptime", millis());
}

void sendRestartAndRestart() {
    webServer.send(200, "application/json", "{\"restart\":true}");
    delay(SecurityConfig::RestartDelayMs);
    ESP.restart();
}

} // namespace Management

#endif
