#include "DisplayWeb.h"
#include "UpdateSafety.h"

#include "DeviceSecrets.h"
#include "DiagnosticLog.h"
#include "DisplayData.h"
#include "DisplayReceiver.h"
#include "ManagementServer.h"
#include "RuntimeSimulation.h"
#include "AuthHelpers.h"
#include "WebAssets.h"

namespace {

void log(const String& message) {
  DiagnosticLog::appendf("[DISPLAY] %s", message.c_str());
}

String diagnosticReport() {
  using namespace DisplayData;
  const auto& rt = runtime();
  String report;
  report.reserve(DiagnosticLog::size() + 1024);
  report += "CANOBD2 display diagnostic report\n";
  report += String("Firmware: ") + ProjectConfig::FirmwareVersion + "\nProtocol: " + ProjectConfig::ProtocolVersion;
  report += "\nUptime: " + String(millis()) + " ms\n";
  report += "Packets ok/lost/crc: " + String(rt.receivedPackets) + "/" + String(rt.droppedPackets) + "/" +
            String(rt.crcErrors) + "\n";
  report += "MAC filtered: " + String(DisplayReceiver::macFilteredCount()) +
            ", queue drops: " + String(DisplayReceiver::queueDropCount()) + "\n";
  report += "Last error: " + rt.lastError + "\n\n----- persistent diagnostic log -----\n";
  report += DiagnosticLog::readAll();
  return report;
}


void handleRoot() {
  if (!WebSecurity::requireAuthentication(Management::server())) return;
  String page(WebAssets::displayPage());
  page.replace("%%COMMON_JS%%", WebAssets::commonScript());
  Management::server().send(200, "text/html; charset=utf-8", page);
}

void handleStatus() {
  using namespace DisplayData;
  if (!WebSecurity::requireAuthentication(Management::server())) return;
  const auto& rt = runtime();
  String json = "{";
  Management::appendCommonStatusJson(json);
  json += "\"espNowConnected\":" + String(isEspNowConnected() ? "true" : "false") + ",";
  json += "\"simulated\":" + String(live().simulated() ? "true" : "false") + ",";
  json += "\"diagnosticPagesEnabled\":" + String(rt.diagnosticPagesEnabled ? "true" : "false") + ",";
  json += "\"visiblePageCount\":" + String(visiblePageCount()) + ",";
  json += "\"currentPage\":" + String(currentPage) + ",";
  json += "\"receivedPackets\":" + String(rt.receivedPackets) + ",";
  json += "\"droppedPackets\":" + String(rt.droppedPackets) + ",";
  json += "\"crcErrors\":" + String(rt.crcErrors) + ",";
  json += "\"lastPacketAgeMs\":" + String(rt.lastReceivedAt == 0 ? 0 : millis() - rt.lastReceivedAt) + ",";
  json += "\"lastHeartbeatAgeMs\":" + String(rt.lastHeartbeatAt == 0 ? 0 : millis() - rt.lastHeartbeatAt) + ",";
  json += "\"vehicleState\":\"" + Management::jsonEscape(rt.vehicleState) + "\",";
  json += "\"lastError\":\"" + Management::jsonEscape(rt.lastError) + "\"}";
  Management::server().send(200, "application/json", json);
}

void handleDiagnosticPagesToggle() {
  using namespace DisplayData;
  if (!WebSecurity::requireAuthentication(Management::server())) return;
  setDiagnosticPagesEnabled(!runtime().diagnosticPagesEnabled);
  Management::server().send(200, "application/json",
                            String("{\"diagnosticPagesEnabled\":") +
                                (runtime().diagnosticPagesEnabled ? "true" : "false") + "}");
}

// Updates must not interrupt the driver: block while the sender reports a
// running vehicle.
bool vehicleActive() {
  return FirmwareUpdate::updateSafety.blocked(millis());
}

} // namespace

namespace DisplayWeb {

void begin() {
#if CANOBD2_ENABLE_DISPLAY_OTA
  const Security::DeviceSecrets& secrets = Security::secrets();
  Management::Config config{};
  config.deviceName = "Display";
  config.apSsid = secrets.displayApSsid;
  config.apPassword = secrets.displayApPassword;
  config.log = log;
  config.diagnosticReport = diagnosticReport;
  config.installBlocker = vehicleActive;
  if (!Management::begin(config)) {
    DisplayData::runtime().lastError = "Nicht provisioniert";
    return;
  }
  WebServer& server = Management::server();
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/api/display/diagnostic-pages/toggle", HTTP_POST, handleDiagnosticPagesToggle);
#endif
}

void handle() {
#if CANOBD2_ENABLE_DISPLAY_OTA
  Management::handle();
#endif
}

} // namespace DisplayWeb
