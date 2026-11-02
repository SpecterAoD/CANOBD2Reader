// Sender web console: device specific pages and APIs on top of the shared
// management server (lib/web/ManagementServer). Common endpoints such as
// signed firmware upload, GitHub updates, WiFi, logs, restart and simulation
// live in the shared module.
#include "WebConsoleHandler.h"

#include <WiFi.h>

#include "DeviceSecrets.h"
#include "DiagnosticLog.h"
#include "Logger.h"
#include "ObdDiagnostics.h"
#include "ManagementServer.h"
#include "RuntimeSimulation.h"
#include "SenderCapabilityScanner.h"
#include "WebAssets.h"
#include "AuthHelpers.h"
#include "config/BuildConfig.h"
#include "config/LoggingConfig.h"
#include "config/NetworkConfig.h"
#include "config/ProjectConfig.h"
#include "config/SenderConfig.h"

namespace {
constexpr size_t MaxLogLineLength = 180;

String clipped(const String& value, size_t maxLength) {
    if (value.length() <= maxLength) return value;
    return value.substring(0, maxLength - 3) + "...";
}

String hex32(uint32_t value, uint8_t width = 8) {
    char buffer[12];
    snprintf(buffer, sizeof(buffer), width == 3 ? "0x%03lX" : "0x%08lX", static_cast<unsigned long>(value));
    return String(buffer);
}

void managementLog(const String& message) {
    WebConsoleHandler::log(message);
}

String managementReport() {
    return WebConsoleHandler::diagnosticTextReport();
}
} // namespace

void WebConsoleHandler::begin() {
#if CANOBD2_ENABLE_SENDER_WEBCONSOLE
    if (!BuildConfig::SenderWebConsoleEnabled) return;

    Logger::setSink([](const char* msg) { WebConsoleHandler::log(String(msg)); });

    const Security::DeviceSecrets& secrets = Security::secrets();
    Management::Config config{};
    config.deviceName = "Sender";
    config.apSsid = secrets.senderApSsid;
    config.apPassword = secrets.senderApPassword;
    config.log = managementLog;
    config.diagnosticReport = managementReport;
    config.installBlocker = installBlocker;
    if (!Management::begin(config)) return;

    WebServer& server = Management::server();
    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/log", HTTP_GET, handleLog);
    server.on("/api/diagnostic/snapshot", HTTP_GET, handleDiagnosticSnapshot);
    server.on("/api/diagnostic/download", HTTP_GET, handleDiagnosticDownload);
    server.on("/start", HTTP_POST, handleStart);
#if CANOBD2_DEV_TOOLS
    server.on("/api/capabilities/status", HTTP_GET, handleCapabilitiesStatus);
    server.on("/api/capabilities/export.json", HTTP_GET, handleCapabilitiesExport);
    server.on("/api/capabilities/obd/start", HTTP_POST, handleCapabilitiesObdStart);
    server.on("/api/capabilities/uds/start", HTTP_POST, handleCapabilitiesUdsStart);
    server.on("/api/capabilities/can/start", HTTP_POST, handleCapabilitiesCanStart);
    server.on("/api/capabilities/can/baseline", HTTP_POST, handleCapabilitiesCanBaseline);
    server.on("/api/capabilities/can/action/start", HTTP_POST, handleCapabilitiesCanActionStart);
    server.on("/api/capabilities/can/action/finish", HTTP_POST, handleCapabilitiesCanActionFinish);
    server.on("/api/capabilities/stop", HTTP_POST, handleCapabilitiesStop);
#endif
#endif
}

void WebConsoleHandler::handle() {
#if CANOBD2_ENABLE_SENDER_WEBCONSOLE
    Management::handle();
#endif
}

void WebConsoleHandler::setInstallBlocker(bool (*blocker)()) {
    installBlocker = blocker;
}

void WebConsoleHandler::log(const String& msg) {
    const String line = clipped(String(millis() / 1000) + "s " + msg, MaxLogLineLength);
    if (logBuffer.size() >= NetworkConfig::WebConsoleMaxLines) logBuffer.pop_front();
    logBuffer.push_back(line);
    if (LoggingConfig::SerialEnabled) Serial.println(msg);
    DiagnosticLog::append(line.c_str());
}

void WebConsoleHandler::recordTelemetry(const char* payload) {
    runtimeStatus.lastTelemetry = clipped(String(payload), 120);
    if (SenderConfig::PersistTelemetryPayloadsToDiagnosticLog) {
        DiagnosticLog::appendf("[TX] %s", payload);
    }
}

void WebConsoleHandler::updateRuntimeStatus(const Runtime::WebRuntimeStatus& status) {
    runtimeStatus = status;
}

void WebConsoleHandler::appendLiveWebBuffer(String& report) {
    if (logBuffer.empty()) return;
    report += "\n----- live web buffer -----\n";
    for (auto& line : logBuffer) {
        report += line;
        report += "\n";
    }
}

void WebConsoleHandler::handleRoot() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    String page(WebAssets::senderPage());
    page.replace("%%COMMON_JS%%", WebAssets::commonScript());
    page.replace("%%SIMULATION_SCENARIOS%%", WebAssets::simulationScenariosJsonArray());
#if !CANOBD2_DEV_TOOLS
    // Release build: hide developer tabs and controls (endpoints do not exist).
    page.replace("<button data-page=\"cap\">Cap</button>", "");
    page.replace("<div class=\"card wide\"><div class=\"label\">Simulationsszenario</div>",
                 "<div class=\"card wide\" style=\"display:none\"><div class=\"label\">Simulationsszenario</div>");
    page.replace("setInterval(refreshCapabilities,5000);", "");
    page.replace("refreshSimulation();refreshCapabilities();", "refreshSimulation();");
#endif
    Management::server().send(200, "text/html", page);
}

void WebConsoleHandler::handleStatus() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    Runtime::WebRuntimeStatus s = runtimeStatus;
    String json;
    json.reserve(1600);
    json += "{";
    Management::appendCommonStatusJson(json);
    json += "\"obdInvalidResponses\":" + String(Obd::Diagnostics::invalidResponseCount()) + ",";
    json += "\"started\":" + String(isStarted() ? "true" : "false") + ",";
    json += "\"canActive\":" + String(s.canActive ? "true" : "false") + ",";
    json += "\"obdActive\":" + String(s.obdActive ? "true" : "false") + ",";
    json += "\"espNowState\":\"" + Management::jsonEscape(s.espNowState) + "\",";
    json += "\"obdState\":\"" + Management::jsonEscape(s.obdState) + "\",";
    json += "\"ledState\":\"" + Management::jsonEscape(s.ledState) + "\",";
    json += "\"ledTestActive\":" + String(s.ledTestActive ? "true" : "false") + ",";
    json += "\"vehicleOff\":" + String(s.vehicleOff ? "true" : "false") + ",";
    json += "\"ledLastStateChangeAt\":" + String(s.ledLastStateChangeAt) + ",";
    json += "\"vehicleState\":\"" + Management::jsonEscape(s.vehicleState) + "\",";
    json += "\"activityScore\":" + String(s.activityScore) + ",";
    json += "\"powerCommand\":\"" + Management::jsonEscape(s.powerCommand) + "\",";
    json += "\"startStopDetected\":" + String(s.startStopDetected ? "true" : "false") + ",";
    json += "\"parkedDetected\":" + String(s.parkedDetected ? "true" : "false") + ",";
    json += "\"parkedStartedAtMs\":" + String(s.parkedStartedAtMs) + ",";
    json += "\"displaySleepDueAtMs\":" + String(s.displaySleepDueAtMs) + ",";
    json += "\"lastWakeupAtMs\":" + String(s.lastWakeupAtMs) + ",";
    json += "\"lastSleepAtMs\":" + String(s.lastSleepAtMs) + ",";
    json += "\"txOk\":" + String(s.telemetrySendOk) + ",";
    json += "\"txFail\":" + String(s.telemetrySendFail) + ",";
    json += "\"heartbeats\":" + String(s.heartbeatCount) + ",";
    json += "\"pidSupport\":" + String(s.pidSupportReady ? "true" : "false") + ",";
    json += "\"supportedPidMask01_20\":\"" + hex32(s.supportedPidMask01_20) + "\",";
    json += "\"supportedPidMask21_40\":\"" + hex32(s.supportedPidMask21_40) + "\",";
    json += "\"supportedPidMask41_60\":\"" + hex32(s.supportedPidMask41_60) + "\",";
    json += "\"simulation\":" + String(s.simulationActive ? "true" : "false") + ",";
    json += "\"simulationScenario\":\"" + Management::jsonEscape(s.simulationScenario) + "\",";
    json += "\"battery\":" + String(s.batteryVoltage, 2) + ",";
    json += "\"seq\":" + String(s.telemetrySequence) + ",";
    json += "\"lastCanAge\":" + String(s.lastCanAgeMs) + ",";
    json += "\"lastObdAge\":" + String(s.lastObdAgeMs) + ",";
    json += "\"canState\":\"" + Management::jsonEscape(s.canState) + "\",";
    json += "\"obdRequestCount\":" + String(s.obdRequestCount) + ",";
    json += "\"obdSendFailureCount\":" + String(s.obdSendFailureCount) + ",";
    json += "\"obdTimeoutCount\":" + String(s.obdTimeoutCount) + ",";
    json += "\"obdValidResponseCount\":" + String(s.obdValidResponseCount) + ",";
    json += "\"obdNegativeResponseCount\":" + String(s.obdNegativeResponseCount) + ",";
    json += "\"obdTimeoutStreak\":" + String(s.obdTimeoutStreak) + ",";
    json += "\"obdPhysicalFallbackActive\":" + String(s.obdPhysicalFallbackActive ? "true" : "false") + ",";
    json += "\"obdRequestCanId\":\"" + hex32(s.obdRequestCanId, 3) + "\",";
    json += "\"lastObdRequest\":\"" + Management::jsonEscape(s.lastObdRequest) + "\",";
    json += "\"lastEcuResponse\":\"" + Management::jsonEscape(s.lastEcuResponse) + "\",";
    json += "\"lastNegativeResponse\":\"" + Management::jsonEscape(s.lastNegativeResponse) + "\",";
    json += "\"udsAvailable\":" + String(s.udsAvailable ? "true" : "false") + ",";
    json += "\"udsRequestCount\":" + String(s.udsRequestCount) + ",";
    json += "\"udsSendFailureCount\":" + String(s.udsSendFailureCount) + ",";
    json += "\"udsTimeoutCount\":" + String(s.udsTimeoutCount) + ",";
    json += "\"udsPositiveResponseCount\":" + String(s.udsPositiveResponseCount) + ",";
    json += "\"udsNegativeResponseCount\":" + String(s.udsNegativeResponseCount) + ",";
    json += "\"lastUdsRequest\":\"" + Management::jsonEscape(s.lastUdsRequest) + "\",";
    json += "\"lastUdsResponse\":\"" + Management::jsonEscape(s.lastUdsResponse) + "\",";
    json += "\"lastUdsNegativeResponse\":\"" + Management::jsonEscape(s.lastUdsNegativeResponse) + "\",";
    json += "\"lastUdsDid\":\"" + Management::jsonEscape(s.lastUdsDid) + "\",";
    json += "\"lastUdsDtc\":\"" + Management::jsonEscape(s.lastUdsDtc) + "\",";
    json += "\"lastSendError\":\"" + Management::jsonEscape(s.lastSendError) + "\",";
    json += "\"lastDtc\":\"" + Management::jsonEscape(s.lastDtc) + "\",";
    json += "\"lastVin\":\"" + Management::jsonEscape(s.lastVin) + "\",";
    json += "\"lastTelemetry\":\"" + Management::jsonEscape(s.lastTelemetry) + "\",";
    json += "\"lastError\":\"" + Management::jsonEscape(s.lastError) + "\"";
    json += "}";
    Management::server().send(200, "application/json", json);
}

String WebConsoleHandler::diagnosticSnapshotJson() {
    Runtime::WebRuntimeStatus s = runtimeStatus;
    String json;
    json.reserve(1800);
    json += "{";
    json += "\"firmware\":\"" + Management::jsonEscape(ProjectConfig::FirmwareVersion) + "\",";
    json += "\"target\":\"" + Management::jsonEscape(ProjectConfig::TargetName) + "\",";
    json += "\"protocol\":" + String(ProjectConfig::ProtocolVersion) + ",";
    json += "\"uptimeMs\":" + String(s.uptimeMs) + ",";
    json += "\"ip\":\"" + Management::jsonEscape(WiFi.softAPIP().toString()) + "\",";
    json += "\"managementAp\":{\"mode\":" + String(static_cast<int>(WiFi.getMode())) +
            ",\"clients\":" + String(WiFi.softAPgetStationNum()) +
            ",\"watchdogMs\":" + String(NetworkConfig::ManagementApWatchdogIntervalMs) + "},";
    json += "\"can\":{\"active\":" + String(s.canActive ? "true" : "false") +
            ",\"state\":\"" + Management::jsonEscape(s.canState) +
            "\",\"lastAgeMs\":" + String(s.lastCanAgeMs) + "},";
    json += "\"espNow\":{\"state\":\"" + Management::jsonEscape(s.espNowState) +
            "\",\"txOk\":" + String(s.telemetrySendOk) +
            ",\"txFail\":" + String(s.telemetrySendFail) +
            ",\"sequence\":" + String(s.telemetrySequence) +
            ",\"heartbeats\":" + String(s.heartbeatCount) + "},";
    json += "\"obd\":{\"active\":" + String(s.obdActive ? "true" : "false") +
            ",\"state\":\"" + Management::jsonEscape(s.obdState) +
            "\",\"lastAgeMs\":" + String(s.lastObdAgeMs) +
            ",\"requestCanId\":\"" + hex32(s.obdRequestCanId, 3) +
            "\",\"physicalFallbackActive\":" + String(s.obdPhysicalFallbackActive ? "true" : "false") +
            ",\"requests\":" + String(s.obdRequestCount) +
            ",\"sendFailures\":" + String(s.obdSendFailureCount) +
            ",\"timeouts\":" + String(s.obdTimeoutCount) +
            ",\"validResponses\":" + String(s.obdValidResponseCount) +
            ",\"negativeResponses\":" + String(s.obdNegativeResponseCount) +
            ",\"timeoutStreak\":" + String(s.obdTimeoutStreak) +
            ",\"lastRequest\":\"" + Management::jsonEscape(s.lastObdRequest) +
            "\",\"lastEcuResponse\":\"" + Management::jsonEscape(s.lastEcuResponse) +
            "\",\"lastNegativeResponse\":\"" + Management::jsonEscape(s.lastNegativeResponse) + "\"},";
    json += "\"uds\":{\"available\":" + String(s.udsAvailable ? "true" : "false") +
            ",\"requests\":" + String(s.udsRequestCount) +
            ",\"sendFailures\":" + String(s.udsSendFailureCount) +
            ",\"timeouts\":" + String(s.udsTimeoutCount) +
            ",\"positiveResponses\":" + String(s.udsPositiveResponseCount) +
            ",\"negativeResponses\":" + String(s.udsNegativeResponseCount) +
            ",\"lastRequest\":\"" + Management::jsonEscape(s.lastUdsRequest) +
            "\",\"lastResponse\":\"" + Management::jsonEscape(s.lastUdsResponse) +
            "\",\"lastNegativeResponse\":\"" + Management::jsonEscape(s.lastUdsNegativeResponse) +
            "\",\"lastDid\":\"" + Management::jsonEscape(s.lastUdsDid) +
            "\",\"lastDtc\":\"" + Management::jsonEscape(s.lastUdsDtc) + "\"},";
    json += "\"supportedPids\":{\"ready\":" + String(s.pidSupportReady ? "true" : "false") +
            ",\"01_20\":\"" + hex32(s.supportedPidMask01_20) +
            "\",\"21_40\":\"" + hex32(s.supportedPidMask21_40) +
            "\",\"41_60\":\"" + hex32(s.supportedPidMask41_60) + "\"},";
    json += "\"vehicle\":{\"vin\":\"" + Management::jsonEscape(s.lastVin) +
            "\",\"dtc\":\"" + Management::jsonEscape(s.lastDtc) +
            "\",\"battery\":" + String(s.batteryVoltage, 2) + "},";
    json += "\"simulation\":{\"active\":" + String(s.simulationActive ? "true" : "false") +
            ",\"scenario\":\"" + Management::jsonEscape(s.simulationScenario) + "\"},";
    json += "\"led\":{\"state\":\"" + Management::jsonEscape(s.ledState) +
            "\",\"testActive\":" + String(s.ledTestActive ? "true" : "false") +
            ",\"vehicleOff\":" + String(s.vehicleOff ? "true" : "false") +
            ",\"lastStateChangeAt\":" + String(s.ledLastStateChangeAt) + "},";
    json += "\"power\":{\"vehicleState\":\"" + Management::jsonEscape(s.vehicleState) +
            "\",\"activityScore\":" + String(s.activityScore) +
            ",\"command\":\"" + Management::jsonEscape(s.powerCommand) +
            "\",\"startStopDetected\":" + String(s.startStopDetected ? "true" : "false") +
            ",\"parkedDetected\":" + String(s.parkedDetected ? "true" : "false") +
            ",\"parkedStartedAtMs\":" + String(s.parkedStartedAtMs) +
            ",\"displaySleepDueAtMs\":" + String(s.displaySleepDueAtMs) +
            ",\"lastWakeupAtMs\":" + String(s.lastWakeupAtMs) +
            ",\"lastSleepAtMs\":" + String(s.lastSleepAtMs) + "},";
    json += "\"lastTelemetry\":\"" + Management::jsonEscape(s.lastTelemetry) + "\",";
    json += "\"lastError\":\"" + Management::jsonEscape(s.lastError) + "\",";
    json += "\"lastSendError\":\"" + Management::jsonEscape(s.lastSendError) + "\"";
    json += "}";
    return json;
}

String WebConsoleHandler::diagnosticTextReport() {
    String report;
    report.reserve(DiagnosticLog::size() + 4096);
    report += "CANOBD2 sender diagnostic report\n";
    report += "Firmware: ";
    report += ProjectConfig::FirmwareVersion;
    report += "\nTarget: ";
    report += ProjectConfig::TargetName;
    report += "\nProtocol: ";
    report += String(ProjectConfig::ProtocolVersion);
    report += "\nUptime: ";
    report += String(millis());
    report += " ms\n";
    report += "Persistent log mounted: ";
    report += DiagnosticLog::mounted() ? "yes" : "no";
    report += "\nPersistent log size: ";
    report += String(static_cast<unsigned long>(DiagnosticLog::size()));
    report += " bytes\n\n";

    report += "----- persistent diagnostic log -----\n";
    report += DiagnosticLog::readAll();
    if (!report.endsWith("\n")) report += "\n";

    appendLiveWebBuffer(report);

    report += "\n----- diagnostic snapshot json -----\n";
    report += diagnosticSnapshotJson();
    report += "\n";
    return report;
}

void WebConsoleHandler::handleLog() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    String page;
    page.reserve(DiagnosticLog::size() + 1024);
    page += DiagnosticLog::readAll();
    if (!page.endsWith("\n")) page += "\n";

    if (!logBuffer.empty()) {
        page += "\n----- live web buffer -----\n";
        for (auto &line : logBuffer) {
            page += line + "\n";
        }
    }
    Management::server().send(200, "text/plain; charset=utf-8", page);
}

void WebConsoleHandler::handleDiagnosticSnapshot() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    Management::server().send(200, "application/json", diagnosticSnapshotJson());
}

void WebConsoleHandler::handleDiagnosticDownload() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    String filename = "CANOBD2_sender_";
    filename += ProjectConfig::FirmwareVersion;
    filename += "_diagnostic_snapshot.json";
    Management::server().sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    handleDiagnosticSnapshot();
}

void WebConsoleHandler::handleStart() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    startRequested = true;
    log("[WebConsole] Start freigegeben");
    Management::server().send(200, "text/plain", "System gestartet");
}

void WebConsoleHandler::handleCapabilitiesStatus() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    Management::server().send(200, "application/json", SenderCapabilityScanner::statusJson());
}

void WebConsoleHandler::handleCapabilitiesExport() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    String filename = "CANOBD2_sender_";
    filename += ProjectConfig::FirmwareVersion;
    filename += "_capabilities.json";
    Management::server().sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    Management::server().send(200, "application/json", SenderCapabilityScanner::exportJson());
}

void WebConsoleHandler::handleCapabilitiesObdStart() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    SenderCapabilityScanner::startObdPidScan();
    Management::server().send(200, "application/json", SenderCapabilityScanner::statusJson());
}

void WebConsoleHandler::handleCapabilitiesUdsStart() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    SenderCapabilityScanner::startUdsScan();
    Management::server().send(200, "application/json", SenderCapabilityScanner::statusJson());
}

void WebConsoleHandler::handleCapabilitiesCanStart() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    SenderCapabilityScanner::startCanSniffer();
    Management::server().send(200, "application/json", SenderCapabilityScanner::statusJson());
}

void WebConsoleHandler::handleCapabilitiesCanBaseline() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    SenderCapabilityScanner::resetCanSnifferBaseline();
    Management::server().send(200, "application/json", SenderCapabilityScanner::statusJson());
}

void WebConsoleHandler::handleCapabilitiesCanActionStart() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    SenderCapabilityScanner::startCanActionCapture();
    Management::server().send(200, "application/json", SenderCapabilityScanner::statusJson());
}

void WebConsoleHandler::handleCapabilitiesCanActionFinish() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    SenderCapabilityScanner::finishCanActionCapture();
    Management::server().send(200, "application/json", SenderCapabilityScanner::statusJson());
}

void WebConsoleHandler::handleCapabilitiesStop() {
    if (!WebSecurity::requireAuthentication(Management::server())) return;
    SenderCapabilityScanner::stop();
    Management::server().send(200, "application/json", SenderCapabilityScanner::statusJson());
}
