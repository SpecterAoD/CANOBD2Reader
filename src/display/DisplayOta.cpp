#include "DisplayOta.h"
#include "DisplayData.h"
#include "Config.h"
#include "RuntimeSimulation.h"
#include "SimulationTypes.h"
#include "AuthHelpers.h"
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

namespace {
  WebServer server(Config::Network::WebServerPort);
  bool webUpdateInProgress = false;
  String webOtaStatus = "Bereit";

  String updateErrorText(const char* prefix) {
    return String(prefix) + ", error=" + String(Update.getError());
  }

  String jsonEscape(const String& value) {
    String escaped;
    escaped.reserve(value.length() + 8);
    for (uint16_t i = 0; i < value.length(); ++i) {
      const char c = value[i];
      if (c == '"' || c == '\\') {
        escaped += '\\';
        escaped += c;
      } else if (c == '\n') {
        escaped += "\\n";
      } else if (c == '\r') {
        escaped += "\\r";
      } else {
        escaped += c;
      }
    }
    return escaped;
  }

  String simulationJson() {
    String json;
    json.reserve(160);
    json += "{";
    json += "\"simulation\":" + String(Simulation::RuntimeSimulation::enabled() ? "true" : "false") + ",";
    json += "\"scenario\":\"" + jsonEscape(Simulation::RuntimeSimulation::scenarioName()) + "\"";
    json += "}";
    return json;
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
<div class="card"><div class="label">Firmware Update</div><div class="small">Nur passende <code>firmware.bin</code> für <b>env:display</b> verwenden. Nach erfolgreichem Upload startet das Display neu.</div></div>
<form method="POST" action="/update" enctype="multipart/form-data">
<input type="file" name="firmware" accept=".bin">
<button type="submit">Display Firmware hochladen</button>
</form>
<div class="card"><div class="label">Firmware</div><div class="small">Version: <span id="fw">--</span><br>Target: <span id="target">--</span><br>Protocol: <span id="proto">--</span><br>Build: <span id="build">--</span></div></div>
<div class="card"><div class="label">OTA Speicher</div><div id="ota" class="value">--</div><div class="small">Sketch: <span id="sketch">--</span> · Flash: <span id="flash">--</span></div></div>
<div class="card"><div class="label">Simulation</div><div id="sim" class="value">--</div><div class="small">Szenario: <span id="scenario">--</span></div><select id="scenarioSelect" style="width:100%;padding:11px;border-radius:12px;background:#0f172a;color:#f8fafc;border:1px solid #263244;margin-top:10px"></select><button id="simToggle" type="button">Simulation umschalten</button><button id="simOn" type="button">Simulation einschalten</button><button id="simOff" type="button">Simulation ausschalten</button></div>
<div class="card"><div class="label">Status</div><div id="status" class="value">--</div><div id="detail" class="small">--</div></div>
<button class="danger" id="restart">Display neu starten</button>
</div>
<script>
const $=id=>document.getElementById(id);
const scenarios=['NormalSingleFrame','NormalMultiFrameVin','NormalMultiFrameDtc','FlowControlRequired','TimeoutAfterFirstFrame','SequenceError','BufferOverflow','MultipleEcusResponse','NegativeResponse','DisplayNormalValues','DisplayWarningValues','DisplayCriticalValues','DisplayTimeoutValues','DisplayMixedValues'];
scenarios.forEach(x=>{let o=document.createElement('option');o.value=x;o.textContent=x;$('scenarioSelect').appendChild(o)});
function bytes(v){return Math.round((v||0)/1024)+' KB'}
async function refresh(){try{let s=await fetch('/status',{cache:'no-store'}).then(r=>r.json());$('ip').textContent=s.ip+' · '+Math.floor(s.uptime/1000)+'s';$('fw').textContent=s.firmware;$('target').textContent=s.target;$('proto').textContent=s.protocol;$('build').textContent=s.buildTime;$('ota').textContent=bytes(s.freeSketchSpace)+' frei';$('sketch').textContent=bytes(s.sketchSize);$('flash').textContent=bytes(s.flashSize);$('status').textContent=s.update?'Update läuft':'Bereit';$('detail').textContent=s.otaStatus+' · '+(s.lastError||'Keine Fehler');$('state').textContent=s.update?'Update':'OK';$('state').className='pill '+(s.update?'err':'ok')}catch(e){$('state').textContent='offline';$('state').className='pill err'}}
async function refreshSimulation(){try{let s=await fetch('/api/simulation',{cache:'no-store'}).then(r=>r.json());$('sim').textContent=s.simulation?'aktiv':'inaktiv';$('scenario').textContent=s.scenario||'--';$('scenarioSelect').value=s.scenario||'NormalSingleFrame'}catch(e){}}
$('restart').onclick=async()=>{if(confirm('Display wirklich neu starten?'))await fetch('/api/restart',{method:'POST'})}
$('simToggle').onclick=async()=>{await fetch('/api/simulation/toggle',{method:'POST'});refreshSimulation()}
$('simOn').onclick=async()=>{await fetch('/api/simulation/on',{method:'POST'});refreshSimulation()}
$('simOff').onclick=async()=>{await fetch('/api/simulation/off',{method:'POST'});refreshSimulation()}
$('scenarioSelect').onchange=async()=>{await fetch('/api/simulation/scenario?scenario='+encodeURIComponent($('scenarioSelect').value),{method:'POST'});refreshSimulation()}
setInterval(refresh,1000);setInterval(refreshSimulation,1000);refresh();refreshSimulation();
</script>
</body></html>
)rawliteral";
    server.send_P(200, "text/html", html);
  }

  void handleStatus() {
    if (!WebSecurity::requireAuthentication(server)) return;
    String json;
    json.reserve(760);
    json += "{";
    json += "\"firmware\":\"" + jsonEscape(Config::Project::FirmwareVersion) + "\",";
    json += "\"target\":\"" + jsonEscape(Config::Project::TargetName) + "\",";
    json += "\"protocol\":" + String(Config::Project::ProtocolVersion) + ",";
    json += "\"buildTime\":\"" + jsonEscape(String(__DATE__) + " " + String(__TIME__)) + "\",";
    json += "\"otaStatus\":\"" + jsonEscape(webOtaStatus) + "\",";
    json += "\"freeSketchSpace\":" + String(ESP.getFreeSketchSpace()) + ",";
    json += "\"sketchSize\":" + String(ESP.getSketchSize()) + ",";
    json += "\"flashSize\":" + String(ESP.getFlashChipSize()) + ",";
    json += "\"ip\":\"" + jsonEscape(WiFi.softAPIP().toString()) + "\",";
    json += "\"uptime\":" + String(millis()) + ",";
    json += "\"update\":" + String(webUpdateInProgress ? "true" : "false") + ",";
    json += "\"simulation\":" + String(Simulation::RuntimeSimulation::enabled() ? "true" : "false") + ",";
    json += "\"simulationScenario\":\"" + jsonEscape(Simulation::RuntimeSimulation::scenarioName()) + "\",";
    json += "\"espNowConnected\":" + String(DisplayData::isEspNowConnected() ? "true" : "false") + ",";
    json += "\"canStatusRecent\":" + String(DisplayData::isCanStatusRecent() ? "true" : "false") + ",";
    json += "\"obdStatusRecent\":" + String(DisplayData::isObdStatusRecent() ? "true" : "false") + ",";
    json += "\"lastPacketAge\":" + String(DisplayData::lastReceivedAt == 0 ? 0 : millis() - DisplayData::lastReceivedAt) + ",";
    json += "\"lastHeartbeatAge\":" + String(DisplayData::lastHeartbeatAt == 0 ? 0 : millis() - DisplayData::lastHeartbeatAt) + ",";
    json += "\"lastHeartbeatSequence\":" + String(DisplayData::lastHeartbeatSequence) + ",";
    json += "\"lastSequence\":" + String(DisplayData::lastSequence) + ",";
    json += "\"receivedPackets\":" + String(DisplayData::receivedPackets) + ",";
    json += "\"droppedPackets\":" + String(DisplayData::droppedPackets) + ",";
    json += "\"crcErrors\":" + String(DisplayData::crcErrors) + ",";
    json += "\"lastError\":\"" + jsonEscape(DisplayData::lastError) + "\"";
    json += "}";
    server.send(200, "application/json", json);
  }

  void handleRestart() {
    if (!WebSecurity::requireAuthentication(server, Config::Security::RequireRestartAuthentication)) return;
    server.send(200, "application/json", "{\"restart\":true}");
    delay(Config::Security::RestartDelayMs);
    ESP.restart();
  }

  void handleSimulationStatus() {
    if (!WebSecurity::requireAuthentication(server, Config::Security::RequireSimulationAuthentication)) return;
    server.send(200, "application/json", simulationJson());
  }

  void handleSimulationOn() {
    if (!WebSecurity::requireAuthentication(server, Config::Security::RequireSimulationAuthentication)) return;
    Simulation::RuntimeSimulation::setEnabled(true);
    DisplayData::lastError = "Simulation eingeschaltet";
    server.send(200, "application/json", simulationJson());
  }

  void handleSimulationOff() {
    if (!WebSecurity::requireAuthentication(server, Config::Security::RequireSimulationAuthentication)) return;
    Simulation::RuntimeSimulation::setEnabled(false);
    DisplayData::lastError = "Simulation ausgeschaltet";
    server.send(200, "application/json", simulationJson());
  }

  void handleSimulationToggle() {
    if (!WebSecurity::requireAuthentication(server, Config::Security::RequireSimulationAuthentication)) return;
    Simulation::RuntimeSimulation::toggle();
    DisplayData::lastError = Simulation::RuntimeSimulation::enabled() ? "Simulation eingeschaltet" : "Simulation ausgeschaltet";
    server.send(200, "application/json", simulationJson());
  }

  void handleSimulationScenario() {
    if (!WebSecurity::requireAuthentication(server, Config::Security::RequireSimulationAuthentication)) return;
    String raw = server.arg("scenario");
    if (raw.length() == 0) raw = server.arg("plain");
    raw.trim();

    Simulation::Scenario scenario;
    if (!Simulation::parseScenario(raw.c_str(), scenario)) {
      server.send(400, "application/json", "{\"error\":\"invalid scenario\"}");
      return;
    }

    Simulation::RuntimeSimulation::setScenario(scenario);
    DisplayData::lastError = "Simulation: " + String(Simulation::RuntimeSimulation::scenarioName());
    server.send(200, "application/json", simulationJson());
  }

  void handleUpdateFinished() {
    if (!WebSecurity::requireAuthentication(server, Config::Security::RequireOtaAuthentication)) return;
    const bool ok = !Update.hasError();
    if (!ok && webOtaStatus == "Bereit") {
      webOtaStatus = updateErrorText("Update fehlgeschlagen");
    }
    server.send(ok ? 200 : 500, "text/plain", ok ? "Update erfolgreich, Neustart..." : webOtaStatus);
    DisplayData::lastError = ok ? "Web-OTA erfolgreich" : webOtaStatus;
    if (ok) {
      webOtaStatus = "Update erfolgreich, Neustart";
      delay(500);
      ESP.restart();
    }
  }

  void handleUpdateUpload() {
    if (!WebSecurity::requireAuthentication(server, Config::Security::RequireOtaAuthentication)) return;
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      webUpdateInProgress = true;
      webOtaStatus = "Upload gestartet: " + upload.filename;
      DisplayData::lastError = webOtaStatus;
      if (Config::Security::RejectOtaWhenSketchSpaceUnknown && ESP.getFreeSketchSpace() == 0) {
        webOtaStatus = "OTA abgelehnt: freier Sketch-Speicher unbekannt";
        DisplayData::lastError = webOtaStatus;
        return;
      }
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        webOtaStatus = updateErrorText("Update.begin fehlgeschlagen");
        DisplayData::lastError = webOtaStatus;
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        webOtaStatus = updateErrorText("Web-OTA Schreibfehler");
        DisplayData::lastError = webOtaStatus;
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        webOtaStatus = "Web-OTA abgeschlossen: " + String(upload.totalSize) + " Bytes";
        DisplayData::lastError = webOtaStatus;
      } else {
        webOtaStatus = updateErrorText("Update.end fehlgeschlagen");
        DisplayData::lastError = webOtaStatus;
      }
      webUpdateInProgress = false;
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      Update.end();
      webUpdateInProgress = false;
      webOtaStatus = "Web-OTA abgebrochen";
      DisplayData::lastError = webOtaStatus;
    }
  }
}

namespace DisplayOta {
  void begin() {
#if !CANOBD2_ENABLE_DISPLAY_OTA
    return;
#endif
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(Config::Network::DisplayWebSsid,
                Config::Network::DisplayWebPassword,
                Config::Network::EspNowChannel);

    ArduinoOTA.setHostname(Config::Network::DisplayOtaHostname);
    ArduinoOTA.setPassword(Config::Network::DisplayWebPassword);
    ArduinoOTA
      .onStart([]() {
        DisplayData::lastError = "OTA gestartet";
      })
      .onEnd([]() {
        DisplayData::lastError = "OTA abgeschlossen";
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        if (total == 0) return;
        DisplayData::lastError = "OTA " + String((progress * 100U) / total) + "%";
      })
      .onError([](ota_error_t error) {
        DisplayData::lastError = "OTA Fehler " + String(static_cast<unsigned int>(error));
      });

    ArduinoOTA.begin();

    server.on("/", HTTP_GET, handleRoot);
    server.on("/status", HTTP_GET, handleStatus);
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
    DisplayData::lastError = "Display Web-OTA bereit";
  }

  void handle() {
#if CANOBD2_ENABLE_DISPLAY_OTA
    ArduinoOTA.handle();
    server.handleClient();
#endif
  }
}
