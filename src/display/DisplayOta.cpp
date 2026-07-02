#include "DisplayOta.h"
#include "DisplayData.h"
#include "common_config.h"
#include "config/ProjectConfig.h"
#include "config/NetworkConfig.h"
#include "config/SecurityConfig.h"
#include "config/LoggingConfig.h"
#include "RuntimeSimulation.h"
#include "SimulationTypes.h"
#include "AuthHelpers.h"
#include "DiagnosticLog.h"
#include "WebAssets.h"
#include "WebRuntimeHandlers.h"
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

namespace {
  WebServer server(NetworkConfig::WebServerPort);
  bool webUpdateInProgress = false;
  String webOtaStatus = "Bereit";

  String simulationJson() {
    return WebRuntimeHandlers::simulationJson();
  }

  void displayWebLogCallback(const String& message) {
    DisplayData::runtime().lastError = message;
    DiagnosticLog::appendf("[DISPLAY] %s", message.c_str());
  }

  void handleRoot() {
    if (!WebSecurity::requireAuthentication(server)) return;
    static const char html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover,user-scalable=no">
<title>CAN OBD2 Display</title>
<style>
:root{color-scheme:dark;--bg:#070b12;--card:#182235;--text:#f8fafc;--muted:#94a3b8;--ok:#22c55e;--warn:#f59e0b;--err:#ef4444;--accent:#38bdf8}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;overflow-x:hidden}
.wrap{width:100%;max-width:430px;margin:0 auto;padding:calc(env(safe-area-inset-top) + 12px) 12px calc(env(safe-area-inset-bottom) + 14px)}
header{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px}.title{font-size:21px;font-weight:800}.sub{font-size:12px;color:var(--muted)}
.pill{padding:6px 10px;border-radius:999px;background:#263244;color:var(--muted);font-size:12px}.pill.ok{background:var(--ok);color:#052e16}.pill.err{background:var(--err);color:#450a0a}
.card{background:var(--card);border:1px solid #263244;border-radius:18px;padding:14px;margin-bottom:10px}.label{font-size:12px;color:var(--muted);margin-bottom:8px}.value{font-size:23px;font-weight:800;word-break:break-word}.small{font-size:13px;color:var(--muted);line-height:1.35}
input[type=file]{width:100%;padding:12px;border-radius:14px;background:#0f172a;color:var(--text);border:1px solid #263244}
button{border:0;border-radius:14px;padding:13px 10px;background:#075985;color:var(--text);font-weight:800;font-size:15px;width:100%;margin-top:10px}.danger{background:#7f1d1d}
</style>
</head>
<body>
<div class="wrap">
<header><div><div class="title">CAN OBD2 Display</div><div class="sub" id="ip">Web OTA</div></div><div id="state" class="pill">bereit</div></header>
<div class="card"><div class="label">Firmware Update</div><div class="small">Nur passende <code>display.bin</code> für <b>env:display</b> verwenden. Dateien mit Sender-Firmware werden abgelehnt. Nach erfolgreichem Upload startet das Display neu.</div></div>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin">
<button type="submit">Display Firmware hochladen</button>
</form>
<div class="card"><div class="label">Firmware</div><div class="small">Version: <span id="fw">--</span><br>Target: <span id="target">--</span><br>Protocol: <span id="proto">--</span><br>Build: <span id="build">--</span></div></div>
<div class="card"><div class="label">OTA Speicher</div><div id="ota" class="value">--</div><div class="small">Sketch: <span id="sketch">--</span> · Flash: <span id="flash">--</span></div></div>
<div class="card"><div class="label">Diagnose-Log</div><div id="diaglog" class="value">--</div><button id="loadLog" type="button">Log anzeigen</button><a href="/log/download"><button type="button">Log herunterladen</button></a><button id="clearLog" class="danger" type="button">Log löschen</button><pre id="logs" style="white-space:pre-wrap;word-break:break-word;background:#020617;color:#86efac;border-radius:12px;padding:10px;max-height:260px;overflow:auto;font-size:12px;display:none"></pre></div>
<div class="card"><div class="label">Simulation</div><div id="sim" class="value">--</div><div class="small">Szenario: <span id="scenario">--</span></div><select id="scenarioSelect" style="width:100%;padding:11px;border-radius:12px;background:#0f172a;color:#f8fafc;border:1px solid #263244;margin-top:10px"></select><button id="simToggle" type="button">Simulation umschalten</button><button id="simOn" type="button">Simulation einschalten</button><button id="simOff" type="button">Simulation ausschalten</button></div>
<div class="card"><div class="label">Status</div><div id="status" class="value">--</div><div id="detail" class="small">--</div></div>
<button class="danger" id="restart">Display neu starten</button>
</div>
<script>
const $=id=>document.getElementById(id);
const scenarios=%%SIMULATION_SCENARIOS%%;
scenarios.forEach(x=>{let o=document.createElement('option');o.value=x;o.textContent=x;$('scenarioSelect').appendChild(o)});
function bytes(v){return Math.round((v||0)/1024)+' KB'}
async function refresh(){try{let s=await fetch('/status',{cache:'no-store'}).then(r=>r.json());$('ip').textContent=s.ip+' · '+Math.floor(s.uptime/1000)+'s';$('fw').textContent=s.firmware;$('target').textContent=s.target;$('proto').textContent=s.protocol;$('build').textContent=s.buildTime;$('ota').textContent=bytes(s.freeSketchSpace)+' frei';$('sketch').textContent=bytes(s.sketchSize);$('flash').textContent=bytes(s.flashSize);$('diaglog').textContent=(s.diagnosticLogMounted?'bereit':'nicht gemountet')+' · '+bytes(s.diagnosticLogSize)+' / '+bytes(s.diagnosticLogMaxSize);$('status').textContent=s.update?'Update läuft':'Bereit';$('detail').textContent=(s.securityWarning||s.otaStatus)+' · '+(s.lastError||'Keine Fehler');$('state').textContent=s.update?'Update':(s.securityReady?'OK':'Warnung');$('state').className='pill '+(s.update?'err':(s.securityReady?'ok':'err'))}catch(e){$('state').textContent='offline';$('state').className='pill err'}}
async function refreshSimulation(){try{let s=await fetch('/api/simulation',{cache:'no-store'}).then(r=>r.json());$('sim').textContent=s.simulation?'aktiv':'inaktiv';$('scenario').textContent=s.scenario||'--';$('scenarioSelect').value=s.scenario||'NormalSingleFrame'}catch(e){}}
async function loadLog(){try{let p=$('logs');p.style.display='block';p.textContent=await fetch('/log/file',{cache:'no-store'}).then(r=>r.text())}catch(e){}}
$('restart').onclick=async()=>{if(confirm('Display wirklich neu starten?'))await fetch('/api/restart',{method:'POST'})}
$('loadLog').onclick=loadLog
$('clearLog').onclick=async()=>{if(confirm('Display Diagnose-Log wirklich löschen?')){await fetch('/api/log/clear',{method:'POST'});refresh();loadLog()}}
$('simToggle').onclick=async()=>{await fetch('/api/simulation/toggle',{method:'POST'});refreshSimulation()}
$('simOn').onclick=async()=>{await fetch('/api/simulation/on',{method:'POST'});refreshSimulation()}
$('simOff').onclick=async()=>{await fetch('/api/simulation/off',{method:'POST'});refreshSimulation()}
$('scenarioSelect').onchange=async()=>{await fetch('/api/simulation/scenario?scenario='+encodeURIComponent($('scenarioSelect').value),{method:'POST'});refreshSimulation()}
setInterval(refresh,1000);setInterval(refreshSimulation,1000);refresh();refreshSimulation();
</script>
</body></html>
)rawliteral";
    String page(html);
    page.replace("%%SIMULATION_SCENARIOS%%", WebAssets::simulationScenariosJsonArray());
    server.send(200, "text/html", page);
  }

  void handleStatus() {
    if (!WebSecurity::requireAuthentication(server)) return;
    String json;
    json.reserve(760);
    json += "{";
    WebRuntimeHandlers::appendFirmwareJson(json, webOtaStatus);
    WebRuntimeHandlers::appendDiagnosticLogJson(json, LoggingConfig::DiagnosticLogMaxBytes);
    json += "\"ip\":\"" + WebRuntimeHandlers::jsonEscape(WiFi.softAPIP().toString()) + "\",";
    json += "\"uptime\":" + String(millis()) + ",";
    json += "\"update\":" + String(webUpdateInProgress ? "true" : "false") + ",";
    json += "\"simulation\":" + String(Simulation::RuntimeSimulation::enabled() ? "true" : "false") + ",";
    json += "\"simulationScenario\":\"" + WebRuntimeHandlers::jsonEscape(Simulation::RuntimeSimulation::scenarioName()) + "\",";
    json += "\"espNowConnected\":" + String(DisplayData::isEspNowConnected() ? "true" : "false") + ",";
    json += "\"canStatusRecent\":" + String(DisplayData::isCanStatusRecent() ? "true" : "false") + ",";
    json += "\"obdStatusRecent\":" + String(DisplayData::isObdStatusRecent() ? "true" : "false") + ",";
    auto& runtime = DisplayData::runtime();
    json += "\"lastPacketAge\":" + String(runtime.lastReceivedAt == 0 ? 0 : millis() - runtime.lastReceivedAt) + ",";
    json += "\"lastHeartbeatAge\":" + String(runtime.lastHeartbeatAt == 0 ? 0 : millis() - runtime.lastHeartbeatAt) + ",";
    json += "\"lastHeartbeatSequence\":" + String(runtime.lastHeartbeatSequence) + ",";
    json += "\"lastSequence\":" + String(runtime.lastSequence) + ",";
    json += "\"receivedPackets\":" + String(runtime.receivedPackets) + ",";
    json += "\"droppedPackets\":" + String(runtime.droppedPackets) + ",";
    json += "\"crcErrors\":" + String(runtime.crcErrors) + ",";
    json += "\"lastError\":\"" + WebRuntimeHandlers::jsonEscape(runtime.lastError) + "\"";
    json += "}";
    server.send(200, "application/json", json);
  }

  void handlePersistentLog() {
    if (!WebSecurity::requireAuthentication(server)) return;
    server.send(200, "text/plain; charset=utf-8", DiagnosticLog::readAll());
  }

  void handleDownloadLog() {
    if (!WebSecurity::requireAuthentication(server)) return;
    String filename = "CANOBD2_display_";
    filename += ProjectConfig::FirmwareVersion;
    filename += "_diagnostic.log";
    server.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    server.send(200, "text/plain; charset=utf-8", DiagnosticLog::readAll());
  }

  void handleClearLog() {
    if (!WebSecurity::requireAuthentication(server)) return;
    const bool ok = DiagnosticLog::clear();
    DiagnosticLog::appendf(ok ? "[DISPLAY] Diagnostic log cleared" : "[DISPLAY] Diagnostic log clear failed");
    server.send(ok ? 200 : 500,
                "application/json",
                ok ? "{\"cleared\":true}" : "{\"cleared\":false}");
  }

  void handleRestart() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireRestartAuthentication)) return;
    DiagnosticLog::appendf("[DISPLAY] Restart requested from web");
    WebRuntimeHandlers::sendRestartResponseAndRestart(server, SecurityConfig::RestartDelayMs, nullptr);
  }

  void handleSimulationStatus() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireSimulationAuthentication)) return;
    server.send(200, "application/json", simulationJson());
  }

  void handleSimulationOn() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireSimulationAuthentication)) return;
    Simulation::RuntimeSimulation::setEnabled(true);
    DisplayData::runtime().lastError = "Simulation eingeschaltet";
    DiagnosticLog::appendf("[DISPLAY] Simulation enabled");
    server.send(200, "application/json", simulationJson());
  }

  void handleSimulationOff() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireSimulationAuthentication)) return;
    Simulation::RuntimeSimulation::setEnabled(false);
    DisplayData::runtime().lastError = "Simulation ausgeschaltet";
    DiagnosticLog::appendf("[DISPLAY] Simulation disabled");
    server.send(200, "application/json", simulationJson());
  }

  void handleSimulationToggle() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireSimulationAuthentication)) return;
    Simulation::RuntimeSimulation::toggle();
    DisplayData::runtime().lastError = Simulation::RuntimeSimulation::enabled() ? "Simulation eingeschaltet" : "Simulation ausgeschaltet";
    DiagnosticLog::appendf("[DISPLAY] Simulation toggled state=%s",
                           Simulation::RuntimeSimulation::enabled() ? "on" : "off");
    server.send(200, "application/json", simulationJson());
  }

  void handleSimulationScenario() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireSimulationAuthentication)) return;
    String raw = server.arg("scenario");
    if (raw.length() == 0) raw = server.arg("plain");
    raw.trim();

    Simulation::Scenario scenario;
    if (!Simulation::parseScenario(raw.c_str(), scenario)) {
      server.send(400, "application/json", "{\"error\":\"invalid scenario\"}");
      return;
    }

    Simulation::RuntimeSimulation::setScenario(scenario);
    DisplayData::runtime().lastError = "Simulation: " + String(Simulation::RuntimeSimulation::scenarioName());
    DiagnosticLog::appendf("[DISPLAY] Simulation scenario=%s", Simulation::RuntimeSimulation::scenarioName());
    server.send(200, "application/json", simulationJson());
  }

  void handleUpdateFinished() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireOtaAuthentication)) return;
    const bool ok = !Update.hasError();
    if (!ok && webOtaStatus == "Bereit") {
      webOtaStatus = WebRuntimeHandlers::updateErrorText("Update fehlgeschlagen");
    }
    server.send(ok ? 200 : 500, "text/plain", ok ? "Update erfolgreich, Neustart..." : webOtaStatus);
    DisplayData::runtime().lastError = ok ? "Web-OTA erfolgreich" : webOtaStatus;
    if (ok) {
      webOtaStatus = "Update erfolgreich, Neustart";
      DiagnosticLog::appendf("[DISPLAY] Web-OTA success, restarting");
      delay(500);
      ESP.restart();
    }
  }

  void handleUpdateUpload() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireOtaAuthentication)) return;
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      webUpdateInProgress = true;
      if (SecurityConfig::RejectOtaWhenSketchSpaceUnknown && ESP.getFreeSketchSpace() == 0) {
        webOtaStatus = "OTA abgelehnt: freier Sketch-Speicher unbekannt";
        DisplayData::runtime().lastError = webOtaStatus;
        DiagnosticLog::appendf("[DISPLAY] Web-OTA rejected: free sketch space unknown");
        return;
      }
      if (!WebRuntimeHandlers::beginWebOtaUpload(upload.filename, webOtaStatus, displayWebLogCallback)) {
        DisplayData::runtime().lastError = webOtaStatus;
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (!WebRuntimeHandlers::writeWebOtaChunk(upload.buf, upload.currentSize, webOtaStatus, displayWebLogCallback)) {
        DisplayData::runtime().lastError = webOtaStatus;
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      WebRuntimeHandlers::finishWebOtaUpload(upload.totalSize, webOtaStatus, displayWebLogCallback);
      DisplayData::runtime().lastError = webOtaStatus;
      webUpdateInProgress = false;
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      WebRuntimeHandlers::abortWebOtaUpload(webOtaStatus, displayWebLogCallback);
      DisplayData::runtime().lastError = webOtaStatus;
      webUpdateInProgress = false;
    }
  }
}

namespace DisplayOta {
  void begin() {
#if !CANOBD2_ENABLE_DISPLAY_OTA
    return;
#endif
    DiagnosticLog::begin();
    DiagnosticLog::appendf("[BOOT] Display firmware=%s protocol=%u target=%s",
                           ProjectConfig::FirmwareVersion,
                           ProjectConfig::ProtocolVersion,
                           ProjectConfig::TargetName);

    const String securityWarning = WebSecurity::displayManagementSecurityWarning();
    if (SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets && securityWarning.length() > 0) {
      webOtaStatus = "Gesperrt: " + securityWarning;
      DisplayData::runtime().lastError = webOtaStatus;
      DiagnosticLog::appendf("[DISPLAY] %s", webOtaStatus.c_str());
      return;
    }

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(NetworkConfig::DisplayWebSsid,
                NetworkConfig::DisplayWebPassword,
                NetworkConfig::EspNowChannel);

    static const char* authHeaders[] = {"X-API-Token", "Authorization"};
    server.collectHeaders(authHeaders, 2);

    ArduinoOTA.setHostname(NetworkConfig::DisplayOtaHostname);
    ArduinoOTA.setPassword(NetworkConfig::DisplayWebPassword);
    ArduinoOTA
      .onStart([]() {
        DisplayData::runtime().lastError = "OTA gestartet";
      })
      .onEnd([]() {
        DisplayData::runtime().lastError = "OTA abgeschlossen";
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        if (total == 0) return;
        DisplayData::runtime().lastError = "OTA " + String((progress * 100U) / total) + "%";
      })
      .onError([](ota_error_t error) {
        DisplayData::runtime().lastError = "OTA Fehler " + String(static_cast<unsigned int>(error));
      });

    ArduinoOTA.begin();

    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/log/file", HTTP_GET, handlePersistentLog);
    server.on("/log/download", HTTP_GET, handleDownloadLog);
    server.on("/api/log/clear", HTTP_POST, handleClearLog);
    server.on("/restart", HTTP_POST, handleRestart);
    server.on("/api/restart", HTTP_POST, handleRestart);
    server.on("/api/simulation", HTTP_GET, handleSimulationStatus);
    server.on("/api/simulation/on", HTTP_POST, handleSimulationOn);
    server.on("/api/simulation/off", HTTP_POST, handleSimulationOff);
    server.on("/api/simulation/toggle", HTTP_POST, handleSimulationToggle);
    server.on("/api/simulation/scenario", HTTP_GET, handleSimulationStatus);
    server.on("/api/simulation/scenario", HTTP_POST, handleSimulationScenario);
    server.on("/update", HTTP_POST, handleUpdateFinished, handleUpdateUpload);
    server.begin();
    DisplayData::runtime().lastError = "Display Web-OTA bereit";
  }

  void handle() {
#if CANOBD2_ENABLE_DISPLAY_OTA
    ArduinoOTA.handle();
    server.handleClient();
#endif
  }
}
