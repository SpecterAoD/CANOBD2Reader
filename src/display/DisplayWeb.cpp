#include "DisplayWeb.h"

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

const char RootPage[] PROGMEM = R"HTML(<!doctype html><html lang="de"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>CANOBD2 Display</title><style>
body{margin:0 auto;max-width:460px;padding:12px;background:#070b12;color:#f8fafc;font-family:-apple-system,BlinkMacSystemFont,Segoe UI,sans-serif}
.card{background:#182235;border:1px solid #263244;border-radius:16px;padding:12px;margin:10px 0}.label{font-size:12px;color:#94a3b8}
.value{font-size:20px;font-weight:700}.small{font-size:13px;color:#94a3b8}a.btn,button{display:block;width:100%;box-sizing:border-box;text-align:center;padding:12px;margin-top:8px;border:0;border-radius:12px;background:#075985;color:#fff;font-weight:700;text-decoration:none;font-size:15px}
.danger{background:#7f1d1d!important}.ok{color:#22c55e}.bad{color:#ef4444}
</style></head><body>
<h2>CANOBD2 Display</h2>
<div class="card"><div class="label">Verbindung zum Sender</div><div id="link" class="value">--</div><div id="packets" class="small">--</div></div>
<div class="card"><div class="label">Firmware</div><div id="fw" class="value">--</div><div id="sec" class="small">--</div>
<a class="btn" href="/github-update">Firmware-Update (signiert)</a></div>
<div class="card"><div class="label">Display-Seiten</div><div id="pages" class="value">--</div>
<button id="toggleDiag">Diagnoseseiten umschalten</button></div>
<div class="card"><div class="label">Diagnose-Log</div><a class="btn" href="/log/download">Log herunterladen</a><button id="clearLog" class="danger">Log loeschen</button></div>
<button id="restart" class="danger">Display neu starten</button>
<script>%%COMMON_JS%%
async function refresh(){try{let s=await (await api('/status')).json();
$('link').textContent=s.espNowConnected?'verbunden':'keine Daten';$('link').className='value '+(s.espNowConnected?'ok':'bad');
$('packets').textContent=s.receivedPackets+' ok, '+s.droppedPackets+' verloren, '+s.crcErrors+' CRC, letztes vor '+(s.lastPacketAgeMs/1000).toFixed(1)+' s'+(s.simulated?' (Simulation)':'');
$('fw').textContent=s.firmware+' / Protokoll '+s.protocol;$('sec').textContent=s.securityReady?'Zugangsdaten: '+s.secretsSource:'Nicht provisioniert: '+s.securityWarning;
$('pages').textContent=s.diagnosticPagesEnabled?'Diagnose ('+s.visiblePageCount+' Seiten)':'Fahrmodus ('+s.visiblePageCount+' Seiten)'}catch(e){$('link').textContent='offline'}}
$('toggleDiag').onclick=async()=>{await post('/api/display/diagnostic-pages/toggle');refresh()};
$('clearLog').onclick=async()=>{if(confirm('Log wirklich loeschen?'))await post('/api/log/clear')};
$('restart').onclick=async()=>{if(confirm('Display neu starten?'))await post('/api/restart')};
refresh();setInterval(refresh,2000);
</script></body></html>)HTML";

void handleRoot() {
  if (!WebSecurity::requireAuthentication(Management::server())) return;
  String page(RootPage);
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
  const String& state = DisplayData::runtime().vehicleState;
  return DisplayData::isEspNowConnected() && (state == "Running" || state == "StartStop" || state == "Idle");
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
