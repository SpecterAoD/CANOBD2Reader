#include "WebConsoleHandler.h"
#include <Update.h>
#include "Logger.h"
#include "RuntimeSimulation.h"
#include "SimulationTypes.h"
#include "AuthHelpers.h"
#include "DiagnosticLog.h"
#include "WebAssets.h"
#include "WebRuntimeHandlers.h"
#include "config/BuildConfig.h"
#include "config/ProjectConfig.h"
#include "config/SenderConfig.h"
#include "config/NetworkConfig.h"
#include "config/SecurityConfig.h"
#include "config/LoggingConfig.h"

namespace {
    constexpr size_t MaxLogLineLength = 180;
    String webOtaStatus = "Bereit";

    String clipped(const String& value, size_t maxLength) {
        if (value.length() <= maxLength) return value;
        return value.substring(0, maxLength - 3) + "...";
    }

    String hex32(uint32_t value, uint8_t width = 8) {
        char buffer[12];
        snprintf(buffer, sizeof(buffer), width == 3 ? "0x%03lX" : "0x%08lX", static_cast<unsigned long>(value));
        return String(buffer);
    }

    void webConsoleLogCallback(const String& message) {
        WebConsoleHandler::log(message);
    }

}

void WebConsoleHandler::begin() {
#if !CANOBD2_ENABLE_SENDER_WEBCONSOLE
    return;
#endif
    if (!BuildConfig::SenderWebConsoleEnabled) return;

    const String securityWarning = WebSecurity::senderManagementSecurityWarning();
    if (SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets && securityWarning.length() > 0) {
        webOtaStatus = "Gesperrt: " + securityWarning;
        DiagnosticLog::appendf("[SECURITY] %s", webOtaStatus.c_str());
        if (LoggingConfig::SerialEnabled) {
            Serial.println(webOtaStatus);
        }
        return;
    }

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(NetworkConfig::SenderWebSsid,
                NetworkConfig::SenderWebPassword,
                NetworkConfig::EspNowChannel);

    Logger::setSink([](const char* msg) {
        WebConsoleHandler::log(String(msg));
    });

    log("[WebConsole] AP gestartet: " + WiFi.softAPIP().toString());

    static const char* authHeaders[] = {"X-API-Token", "Authorization"};
    server.collectHeaders(authHeaders, 2);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/log", HTTP_GET, handleLog);
    server.on("/log/file", HTTP_GET, handlePersistentLog);
    server.on("/log/download", HTTP_GET, handleDownloadLog);
    server.on("/api/log/clear", HTTP_POST, handleClearLog);
    server.on("/api/diagnostic/snapshot", HTTP_GET, handleDiagnosticSnapshot);
    server.on("/api/diagnostic/download", HTTP_GET, handleDiagnosticDownload);
    server.on("/status", HTTP_GET, handleStatus);
    server.on("/start", HTTP_POST, handleStart);
    server.on("/restart", HTTP_POST, handleRestart);
    server.on("/api/restart", HTTP_POST, handleApiRestart);
    server.on("/api/simulation", HTTP_GET, handleSimulationStatus);
    server.on("/api/simulation/on", HTTP_POST, handleSimulationOn);
    server.on("/api/simulation/off", HTTP_POST, handleSimulationOff);
    server.on("/api/simulation/toggle", HTTP_POST, handleSimulationToggle);
    server.on("/api/simulation/scenario", HTTP_GET, handleSimulationStatus);
    server.on("/api/simulation/scenario", HTTP_POST, handleSimulationScenario);
    server.on("/update", HTTP_GET, handleUpdatePage);
    server.on("/update", HTTP_POST, handleUpdateFinished, handleUpdateUpload);
    server.begin();
}

void WebConsoleHandler::handle() {
#if !CANOBD2_ENABLE_SENDER_WEBCONSOLE
    return;
#endif
    if (BuildConfig::SenderWebConsoleEnabled) {
        server.handleClient();
    }
}

void WebConsoleHandler::log(const String& msg) {
#if !CANOBD2_ENABLE_SENDER_WEBCONSOLE
    if (LoggingConfig::SerialEnabled) Serial.println(msg);
    return;
#endif
    const String line = clipped(String(millis() / 1000) + "s " + msg, MaxLogLineLength);
    if (logBuffer.size() >= NetworkConfig::WebConsoleMaxLines) {
        logBuffer.pop_front();
    }
    logBuffer.push_back(line);

    if (LoggingConfig::SerialEnabled) {
        Serial.println(msg);
    }

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

String WebConsoleHandler::jsonEscape(const String& value) {
    return WebRuntimeHandlers::jsonEscape(value);
}

void WebConsoleHandler::handleRoot() {
    if (!WebSecurity::requireAuthentication(server)) return;
    static const char html[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="de">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover,user-scalable=no">
<title>CAN OBD2 Sender</title>
<style>
:root{color-scheme:dark;--bg:#070b12;--panel:#111827;--card:#182235;--text:#f8fafc;--muted:#94a3b8;--ok:#22c55e;--warn:#f59e0b;--err:#ef4444;--accent:#38bdf8}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;overflow-x:hidden}
.wrap{width:100%;max-width:430px;margin:0 auto;padding:calc(env(safe-area-inset-top) + 10px) 12px calc(env(safe-area-inset-bottom) + 14px)}
header{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}.title{font-size:20px;font-weight:800}.sub{font-size:12px;color:var(--muted)}
.pill{padding:6px 10px;border-radius:999px;background:#263244;color:var(--muted);font-size:12px}.pill.ok{color:#052e16;background:var(--ok)}.pill.warn{color:#431407;background:var(--warn)}.pill.err{color:#450a0a;background:var(--err)}
.nav{display:grid;grid-template-columns:repeat(4,1fr);gap:7px;position:sticky;top:0;background:rgba(7,11,18,.92);backdrop-filter:blur(10px);padding:8px 0;z-index:2}
button,.btn{border:0;border-radius:14px;padding:11px 8px;background:#1f2937;color:var(--text);font-weight:700;font-size:13px;text-decoration:none;text-align:center}
button.active{background:var(--accent);color:#042f3d}.page{display:none}.page.active{display:block}.grid{display:grid;grid-template-columns:1fr 1fr;gap:9px}.card{background:var(--card);border:1px solid #263244;border-radius:18px;padding:12px;min-height:78px}.wide{grid-column:1/-1}.label{font-size:12px;color:var(--muted);margin-bottom:8px}.value{font-size:24px;font-weight:800;word-break:break-word}.small{font-size:13px;color:var(--muted);line-height:1.35}.okText{color:var(--ok)}.warnText{color:var(--warn)}.errText{color:var(--err)}
pre{white-space:pre-wrap;word-break:break-word;background:#020617;color:#86efac;border-radius:16px;padding:12px;min-height:250px;max-height:58vh;overflow:auto;font-size:12px}
input[type=file]{width:100%;padding:12px;border-radius:14px;background:#0f172a;color:var(--text);border:1px solid #263244}.actions{display:grid;gap:9px;margin-top:10px}.danger{background:#7f1d1d}.primary{background:#075985}
</style>
</head>
<body>
<div class="wrap">
<header><div><div class="title">CAN OBD2 Sender</div><div class="sub" id="ip">WebConsole</div></div><div id="run" class="pill warn">Warte</div></header>
<nav class="nav">
<button data-page="dash" class="active">Status</button>
<button data-page="diag">Diag</button>
<button data-page="log">Log</button>
<button data-page="ota">OTA</button>
</nav>
<section id="dash" class="page active"><div class="grid">
<div class="card wide"><div class="label">Firmware</div><div class="small">Version: <span id="fw">--</span><br>Target: <span id="target">--</span><br>Protocol: <span id="proto">--</span><br>Build: <span id="build">--</span></div></div>
<div class="card"><div class="label">CAN</div><div id="can" class="value">--</div></div>
<div class="card"><div class="label">OBD2</div><div id="obd" class="value">--</div></div>
<div class="card"><div class="label">ESP-NOW</div><div id="espnow" class="value">--</div></div>
<div class="card"><div class="label">Heartbeat</div><div id="heartbeat" class="value">--</div></div>
<div class="card"><div class="label">Batterie</div><div id="bat" class="value">--</div></div>
<div class="card"><div class="label">Telemetrie</div><div id="seq" class="value">--</div></div>
<div class="card wide"><div class="label">Letztes Paket</div><div id="tel" class="small">--</div></div>
</div><div class="actions"><button id="start" class="primary">Sender starten</button></div></section>
<section id="diag" class="page"><div class="grid">
<div class="card"><div class="label">PID Support</div><div id="pid" class="value">--</div></div>
<div class="card"><div class="label">Letzter CAN</div><div id="canage" class="value">--</div></div>
<div class="card wide"><div class="label">Simulation</div><div id="sim" class="value">--</div><div class="small">Szenario: <span id="scenario">--</span></div></div>
<div class="card wide"><div class="label">Simulationsszenario</div><select id="scenarioSelect" style="width:100%;padding:11px;border-radius:12px;background:#0f172a;color:#f8fafc;border:1px solid #263244"></select><div class="actions"><button id="simToggle">Simulation umschalten</button><button id="simOn" class="primary">Simulation einschalten</button><button id="simOff">Simulation ausschalten</button></div></div>
<div class="card wide"><div class="label">DTC / Fehlercodes</div><div id="dtc" class="value">--</div></div>
<div class="card wide"><div class="label">VIN</div><div id="vin" class="value">--</div></div>
<div class="card wide"><div class="label">OBD Diagnose</div><div class="small">
Requests: <span id="obdReq">--</span> · OK: <span id="obdOk">--</span> · Timeouts: <span id="obdTimeouts">--</span><br>
Negative: <span id="obdNeg">--</span> · SendFail: <span id="obdSendFail">--</span> · Timeout-Streak: <span id="obdStreak">--</span><br>
Request-ID: <span id="obdReqId">--</span> · Fallback: <span id="obdFallback">--</span>
</div></div>
<div class="card wide"><div class="label">Letzte OBD Anfrage</div><div id="lastObdReq" class="small">--</div></div>
<div class="card wide"><div class="label">Letzte ECU Antwort</div><div id="lastEcuResp" class="small">--</div></div>
<div class="card wide"><div class="label">Letzte Negative Response</div><div id="lastNrc" class="small">--</div></div>
<div class="card wide"><div class="label">Supported PID Masks</div><div class="small">01-20: <span id="pidMask1">--</span><br>21-40: <span id="pidMask2">--</span><br>41-60: <span id="pidMask3">--</span></div></div>
<div class="card wide"><div class="label">UDS Diagnose</div><div class="small">
Verfuegbar: <span id="udsAvail">--</span> · Requests: <span id="udsReq">--</span> · OK: <span id="udsOk">--</span><br>
Timeouts: <span id="udsTimeouts">--</span> · Negative: <span id="udsNeg">--</span> · SendFail: <span id="udsSendFail">--</span>
</div></div>
<div class="card wide"><div class="label">Letzte UDS Anfrage</div><div id="lastUdsReq" class="small">--</div></div>
<div class="card wide"><div class="label">Letzte UDS Antwort</div><div id="lastUdsResp" class="small">--</div></div>
<div class="card wide"><div class="label">UDS DID / DTC</div><div class="small">DID: <span id="udsDid">--</span><br>DTC: <span id="udsDtc">--</span><br>NRC: <span id="udsNrc">--</span></div></div>
<div class="card wide"><div class="label">Diagnose-Snapshot</div><div class="actions"><a class="btn primary" href="/api/diagnostic/download">JSON herunterladen</a><button id="loadSnapshot">Snapshot anzeigen</button></div><pre id="snapshot">Noch kein Snapshot geladen.</pre></div>
<div class="card wide"><div class="label">Fehlerstatus</div><div id="err" class="small">--</div></div>
</div></section>
<section id="log" class="page"><div class="grid">
<div class="card wide"><div class="label">Diagnose-Log</div><div class="small">Persistent: <span id="diaglog">--</span></div><div class="actions"><button id="loadFileLog" class="primary">Persistenten Log laden</button><a class="btn primary" href="/log/download">Log herunterladen</a><button id="clearLog" class="danger">Log löschen</button></div></div>
<div class="card wide"><div class="label">Live-Log</div><pre id="logs">Lade Log...</pre></div>
</div></section>
<section id="ota" class="page"><div class="card wide"><div class="label">Firmware über Web hochladen</div><div class="small">Nur passende <code>sender.bin</code> für <b>env:sender</b> verwenden. Dateien mit Display-Firmware werden abgelehnt. Gerät startet nach erfolgreichem Update neu.</div></div>
<div class="card wide"><div class="label">OTA Status</div><div id="otastatus" class="value">--</div><div class="small">Frei: <span id="freeota">--</span> · Sketch: <span id="sketch">--</span> · Flash: <span id="flash">--</span></div></div>
<form class="actions" method="POST" action="/update" enctype="multipart/form-data"><input type="file" name="firmware" accept=".bin"><button class="primary" type="submit">OTA Update starten</button></form>
<div class="actions"><button id="restart" class="danger">Sender neu starten</button></div></section>
</div>
<script>
const $=id=>document.getElementById(id);
const scenarios=%%SIMULATION_SCENARIOS%%;
scenarios.forEach(x=>{let o=document.createElement('option');o.value=x;o.textContent=x;$('scenarioSelect').appendChild(o)});
document.querySelectorAll('.nav button').forEach(b=>b.onclick=()=>{document.querySelectorAll('.nav button').forEach(x=>x.classList.remove('active'));document.querySelectorAll('.page').forEach(x=>x.classList.remove('active'));b.classList.add('active');$(b.dataset.page).classList.add('active')});
function yn(v){return v?'aktiv':'aus'}function cls(el,state){el.className='value '+(state?'okText':'warnText')}function bytes(v){return Math.round((v||0)/1024)+' KB'}
async function refresh(){try{let s=await fetch('/status',{cache:'no-store'}).then(r=>r.json());$('ip').textContent=s.ip+' · '+Math.floor(s.uptime/1000)+'s';$('fw').textContent=s.firmware;$('target').textContent=s.target;$('proto').textContent=s.protocol;$('build').textContent=s.buildTime;$('otastatus').textContent=s.otaStatus;$('freeota').textContent=bytes(s.freeSketchSpace);$('sketch').textContent=bytes(s.sketchSize);$('flash').textContent=bytes(s.flashSize);$('diaglog').textContent=(s.diagnosticLogMounted?'bereit':'nicht gemountet')+' · '+bytes(s.diagnosticLogSize)+' / '+bytes(s.diagnosticLogMaxSize);$('run').textContent=s.started?'läuft':'gestoppt';$('run').className='pill '+(s.started?(s.securityReady?'ok':'warn'):'warn');$('can').textContent=s.canState;cls($('can'),s.canActive);$('obd').textContent=s.obdState||yn(s.obdActive);cls($('obd'),s.obdActive);$('espnow').textContent=s.espNowState;cls($('espnow'),s.espNowState==='READY');$('heartbeat').textContent=s.heartbeats;$('bat').textContent=s.battery.toFixed(2)+' V';$('seq').textContent=s.seq;$('tel').textContent=s.lastTelemetry;$('pid').textContent=s.pidSupport?'bereit':'unbekannt';cls($('pid'),s.pidSupport);$('canage').textContent=(s.lastCanAge/1000).toFixed(1)+'s';$('dtc').textContent=s.lastDtc||'--';$('vin').textContent=s.lastVin||'--';$('obdReq').textContent=s.obdRequestCount;$('obdOk').textContent=s.obdValidResponseCount;$('obdTimeouts').textContent=s.obdTimeoutCount;$('obdNeg').textContent=s.obdNegativeResponseCount;$('obdSendFail').textContent=s.obdSendFailureCount;$('obdStreak').textContent=s.obdTimeoutStreak;$('obdReqId').textContent=s.obdRequestCanId;$('obdFallback').textContent=s.obdPhysicalFallbackActive?'0x7E0 aktiv':'0x7DF';$('lastObdReq').textContent=s.lastObdRequest||'--';$('lastEcuResp').textContent=s.lastEcuResponse||'--';$('lastNrc').textContent=s.lastNegativeResponse||'--';$('pidMask1').textContent=s.supportedPidMask01_20;$('pidMask2').textContent=s.supportedPidMask21_40;$('pidMask3').textContent=s.supportedPidMask41_60;$('udsAvail').textContent=s.udsAvailable?'ja':'nein';$('udsReq').textContent=s.udsRequestCount;$('udsOk').textContent=s.udsPositiveResponseCount;$('udsTimeouts').textContent=s.udsTimeoutCount;$('udsNeg').textContent=s.udsNegativeResponseCount;$('udsSendFail').textContent=s.udsSendFailureCount;$('lastUdsReq').textContent=s.lastUdsRequest||'--';$('lastUdsResp').textContent=s.lastUdsResponse||'--';$('udsDid').textContent=s.lastUdsDid||'--';$('udsDtc').textContent=s.lastUdsDtc||'--';$('udsNrc').textContent=s.lastUdsNegativeResponse||'--';$('err').textContent=s.securityWarning||s.lastSendError||s.lastError||'OK';}catch(e){$('run').textContent='offline';$('run').className='pill err'}}
async function refreshLog(){try{$('logs').textContent=await fetch('/log',{cache:'no-store'}).then(r=>r.text())}catch(e){}}
async function loadPersistentLog(){try{$('logs').textContent=await fetch('/log/file',{cache:'no-store'}).then(r=>r.text())}catch(e){}}
async function refreshSimulation(){try{let s=await fetch('/api/simulation',{cache:'no-store'}).then(r=>r.json());$('sim').textContent=s.simulation?'aktiv':'inaktiv';$('sim').className='value '+(s.simulation?'okText':'warnText');$('scenario').textContent=s.scenario||'--';$('scenarioSelect').value=s.scenario||'NormalSingleFrame'}catch(e){}}
async function loadSnapshot(){try{$('snapshot').textContent=JSON.stringify(await fetch('/api/diagnostic/snapshot',{cache:'no-store'}).then(r=>r.json()),null,2)}catch(e){$('snapshot').textContent='Snapshot konnte nicht geladen werden'}}
$('start').onclick=async()=>{await fetch('/start',{method:'POST'});refresh()}
$('restart').onclick=async()=>{if(confirm('Sender wirklich neu starten?'))await fetch('/api/restart',{method:'POST'})}
$('simToggle').onclick=async()=>{await fetch('/api/simulation/toggle',{method:'POST'});refreshSimulation()}
$('simOn').onclick=async()=>{await fetch('/api/simulation/on',{method:'POST'});refreshSimulation()}
$('simOff').onclick=async()=>{await fetch('/api/simulation/off',{method:'POST'});refreshSimulation()}
$('scenarioSelect').onchange=async()=>{await fetch('/api/simulation/scenario?scenario='+encodeURIComponent($('scenarioSelect').value),{method:'POST'});refreshSimulation()}
$('loadFileLog').onclick=loadPersistentLog
$('loadSnapshot').onclick=loadSnapshot
$('clearLog').onclick=async()=>{if(confirm('Diagnose-Log wirklich löschen?')){await fetch('/api/log/clear',{method:'POST'});refreshLog();refresh()}}
setInterval(refresh,1000);setInterval(refreshLog,1500);setInterval(refreshSimulation,1000);refresh();refreshLog();refreshSimulation();
</script>
</body></html>
)rawliteral";
    String page(html);
    page.replace("%%SIMULATION_SCENARIOS%%", WebAssets::simulationScenariosJsonArray());
    server.send(200, "text/html", page);
}

void WebConsoleHandler::handleLog() {
    if (!WebSecurity::requireAuthentication(server)) return;
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
    server.send(200, "text/plain; charset=utf-8", page);
}

void WebConsoleHandler::handlePersistentLog() {
    if (!WebSecurity::requireAuthentication(server)) return;
    server.send(200, "text/plain; charset=utf-8", DiagnosticLog::readAll());
}

void WebConsoleHandler::handleDownloadLog() {
    if (!WebSecurity::requireAuthentication(server)) return;
    String filename = "CANOBD2_sender_";
    filename += ProjectConfig::FirmwareVersion;
    filename += "_diagnostic.log";
    server.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    server.send(200, "text/plain; charset=utf-8", DiagnosticLog::readAll());
}

void WebConsoleHandler::handleClearLog() {
    if (!WebSecurity::requireAuthentication(server)) return;
    logBuffer.clear();
    const bool ok = DiagnosticLog::clear();
    log(String("[WebConsole] Diagnose-Log ") + (ok ? "geloescht" : "konnte nicht geloescht werden"));
    server.send(ok ? 200 : 500,
                "application/json",
                ok ? "{\"cleared\":true}" : "{\"cleared\":false}");
}

void WebConsoleHandler::handleStatus() {
    if (!WebSecurity::requireAuthentication(server)) return;
    Runtime::WebRuntimeStatus s = runtimeStatus;
    String json;
    json.reserve(1600);
    json += "{";
    WebRuntimeHandlers::appendFirmwareJson(json, webOtaStatus);
    WebRuntimeHandlers::appendDiagnosticLogJson(json, SenderConfig::DiagnosticLogMaxBytes);
    json += "\"started\":" + String(isStarted() ? "true" : "false") + ",";
    json += "\"ip\":\"" + jsonEscape(WiFi.softAPIP().toString()) + "\",";
    json += "\"uptime\":" + String(s.uptimeMs) + ",";
    json += "\"canActive\":" + String(s.canActive ? "true" : "false") + ",";
    json += "\"obdActive\":" + String(s.obdActive ? "true" : "false") + ",";
    json += "\"espNowState\":\"" + jsonEscape(s.espNowState) + "\",";
    json += "\"obdState\":\"" + jsonEscape(s.obdState) + "\",";
    json += "\"txOk\":" + String(s.telemetrySendOk) + ",";
    json += "\"txFail\":" + String(s.telemetrySendFail) + ",";
    json += "\"heartbeats\":" + String(s.heartbeatCount) + ",";
    json += "\"pidSupport\":" + String(s.pidSupportReady ? "true" : "false") + ",";
    json += "\"supportedPidMask01_20\":\"" + hex32(s.supportedPidMask01_20) + "\",";
    json += "\"supportedPidMask21_40\":\"" + hex32(s.supportedPidMask21_40) + "\",";
    json += "\"supportedPidMask41_60\":\"" + hex32(s.supportedPidMask41_60) + "\",";
    json += "\"simulation\":" + String(s.simulationActive ? "true" : "false") + ",";
    json += "\"simulationScenario\":\"" + jsonEscape(s.simulationScenario) + "\",";
    json += "\"battery\":" + String(s.batteryVoltage, 2) + ",";
    json += "\"seq\":" + String(s.telemetrySequence) + ",";
    json += "\"lastCanAge\":" + String(s.lastCanAgeMs) + ",";
    json += "\"lastObdAge\":" + String(s.lastObdAgeMs) + ",";
    json += "\"canState\":\"" + jsonEscape(s.canState) + "\",";
    json += "\"obdRequestCount\":" + String(s.obdRequestCount) + ",";
    json += "\"obdSendFailureCount\":" + String(s.obdSendFailureCount) + ",";
    json += "\"obdTimeoutCount\":" + String(s.obdTimeoutCount) + ",";
    json += "\"obdValidResponseCount\":" + String(s.obdValidResponseCount) + ",";
    json += "\"obdNegativeResponseCount\":" + String(s.obdNegativeResponseCount) + ",";
    json += "\"obdTimeoutStreak\":" + String(s.obdTimeoutStreak) + ",";
    json += "\"obdPhysicalFallbackActive\":" + String(s.obdPhysicalFallbackActive ? "true" : "false") + ",";
    json += "\"obdRequestCanId\":\"" + hex32(s.obdRequestCanId, 3) + "\",";
    json += "\"lastObdRequest\":\"" + jsonEscape(s.lastObdRequest) + "\",";
    json += "\"lastEcuResponse\":\"" + jsonEscape(s.lastEcuResponse) + "\",";
    json += "\"lastNegativeResponse\":\"" + jsonEscape(s.lastNegativeResponse) + "\",";
    json += "\"udsAvailable\":" + String(s.udsAvailable ? "true" : "false") + ",";
    json += "\"udsRequestCount\":" + String(s.udsRequestCount) + ",";
    json += "\"udsSendFailureCount\":" + String(s.udsSendFailureCount) + ",";
    json += "\"udsTimeoutCount\":" + String(s.udsTimeoutCount) + ",";
    json += "\"udsPositiveResponseCount\":" + String(s.udsPositiveResponseCount) + ",";
    json += "\"udsNegativeResponseCount\":" + String(s.udsNegativeResponseCount) + ",";
    json += "\"lastUdsRequest\":\"" + jsonEscape(s.lastUdsRequest) + "\",";
    json += "\"lastUdsResponse\":\"" + jsonEscape(s.lastUdsResponse) + "\",";
    json += "\"lastUdsNegativeResponse\":\"" + jsonEscape(s.lastUdsNegativeResponse) + "\",";
    json += "\"lastUdsDid\":\"" + jsonEscape(s.lastUdsDid) + "\",";
    json += "\"lastUdsDtc\":\"" + jsonEscape(s.lastUdsDtc) + "\",";
    json += "\"lastSendError\":\"" + jsonEscape(s.lastSendError) + "\",";
    json += "\"lastDtc\":\"" + jsonEscape(s.lastDtc) + "\",";
    json += "\"lastVin\":\"" + jsonEscape(s.lastVin) + "\",";
    json += "\"lastTelemetry\":\"" + jsonEscape(s.lastTelemetry) + "\",";
    json += "\"lastError\":\"" + jsonEscape(s.lastError) + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void WebConsoleHandler::handleDiagnosticSnapshot() {
    if (!WebSecurity::requireAuthentication(server)) return;
    Runtime::WebRuntimeStatus s = runtimeStatus;
    String json;
    json.reserve(1800);
    json += "{";
    json += "\"firmware\":\"" + jsonEscape(ProjectConfig::FirmwareVersion) + "\",";
    json += "\"target\":\"" + jsonEscape(ProjectConfig::TargetName) + "\",";
    json += "\"protocol\":" + String(ProjectConfig::ProtocolVersion) + ",";
    json += "\"uptimeMs\":" + String(s.uptimeMs) + ",";
    json += "\"ip\":\"" + jsonEscape(WiFi.softAPIP().toString()) + "\",";
    json += "\"can\":{\"active\":" + String(s.canActive ? "true" : "false") +
            ",\"state\":\"" + jsonEscape(s.canState) +
            "\",\"lastAgeMs\":" + String(s.lastCanAgeMs) + "},";
    json += "\"espNow\":{\"state\":\"" + jsonEscape(s.espNowState) +
            "\",\"txOk\":" + String(s.telemetrySendOk) +
            ",\"txFail\":" + String(s.telemetrySendFail) +
            ",\"sequence\":" + String(s.telemetrySequence) +
            ",\"heartbeats\":" + String(s.heartbeatCount) + "},";
    json += "\"obd\":{\"active\":" + String(s.obdActive ? "true" : "false") +
            ",\"state\":\"" + jsonEscape(s.obdState) +
            "\",\"lastAgeMs\":" + String(s.lastObdAgeMs) +
            ",\"requestCanId\":\"" + hex32(s.obdRequestCanId, 3) +
            "\",\"physicalFallbackActive\":" + String(s.obdPhysicalFallbackActive ? "true" : "false") +
            ",\"requests\":" + String(s.obdRequestCount) +
            ",\"sendFailures\":" + String(s.obdSendFailureCount) +
            ",\"timeouts\":" + String(s.obdTimeoutCount) +
            ",\"validResponses\":" + String(s.obdValidResponseCount) +
            ",\"negativeResponses\":" + String(s.obdNegativeResponseCount) +
            ",\"timeoutStreak\":" + String(s.obdTimeoutStreak) +
            ",\"lastRequest\":\"" + jsonEscape(s.lastObdRequest) +
            "\",\"lastEcuResponse\":\"" + jsonEscape(s.lastEcuResponse) +
            "\",\"lastNegativeResponse\":\"" + jsonEscape(s.lastNegativeResponse) + "\"},";
    json += "\"uds\":{\"available\":" + String(s.udsAvailable ? "true" : "false") +
            ",\"requests\":" + String(s.udsRequestCount) +
            ",\"sendFailures\":" + String(s.udsSendFailureCount) +
            ",\"timeouts\":" + String(s.udsTimeoutCount) +
            ",\"positiveResponses\":" + String(s.udsPositiveResponseCount) +
            ",\"negativeResponses\":" + String(s.udsNegativeResponseCount) +
            ",\"lastRequest\":\"" + jsonEscape(s.lastUdsRequest) +
            "\",\"lastResponse\":\"" + jsonEscape(s.lastUdsResponse) +
            "\",\"lastNegativeResponse\":\"" + jsonEscape(s.lastUdsNegativeResponse) +
            "\",\"lastDid\":\"" + jsonEscape(s.lastUdsDid) +
            "\",\"lastDtc\":\"" + jsonEscape(s.lastUdsDtc) + "\"},";
    json += "\"supportedPids\":{\"ready\":" + String(s.pidSupportReady ? "true" : "false") +
            ",\"01_20\":\"" + hex32(s.supportedPidMask01_20) +
            "\",\"21_40\":\"" + hex32(s.supportedPidMask21_40) +
            "\",\"41_60\":\"" + hex32(s.supportedPidMask41_60) + "\"},";
    json += "\"vehicle\":{\"vin\":\"" + jsonEscape(s.lastVin) +
            "\",\"dtc\":\"" + jsonEscape(s.lastDtc) +
            "\",\"battery\":" + String(s.batteryVoltage, 2) + "},";
    json += "\"simulation\":{\"active\":" + String(s.simulationActive ? "true" : "false") +
            ",\"scenario\":\"" + jsonEscape(s.simulationScenario) + "\"},";
    json += "\"lastTelemetry\":\"" + jsonEscape(s.lastTelemetry) + "\",";
    json += "\"lastError\":\"" + jsonEscape(s.lastError) + "\",";
    json += "\"lastSendError\":\"" + jsonEscape(s.lastSendError) + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void WebConsoleHandler::handleDiagnosticDownload() {
    if (!WebSecurity::requireAuthentication(server)) return;
    String filename = "CANOBD2_sender_";
    filename += ProjectConfig::FirmwareVersion;
    filename += "_diagnostic_snapshot.json";
    server.sendHeader("Content-Disposition", "attachment; filename=\"" + filename + "\"");
    handleDiagnosticSnapshot();
}

void WebConsoleHandler::handleStart() {
    if (!WebSecurity::requireAuthentication(server)) return;
    startRequested = true;
    log("[WebConsole] Start freigegeben");
    server.send(200, "text/plain", "System gestartet");
}

void WebConsoleHandler::handleRestart() {
    handleApiRestart();
}

void WebConsoleHandler::handleApiRestart() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireRestartAuthentication)) return;
    WebRuntimeHandlers::sendRestartResponseAndRestart(server, SecurityConfig::RestartDelayMs, [](const String& message) {
        WebConsoleHandler::log(message);
    });
}

String WebConsoleHandler::simulationJson() {
    return WebRuntimeHandlers::simulationJson();
}

void WebConsoleHandler::handleSimulationStatus() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireSimulationAuthentication)) return;
    server.send(200, "application/json", simulationJson());
}

void WebConsoleHandler::handleSimulationOn() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireSimulationAuthentication)) return;
    Simulation::RuntimeSimulation::setEnabled(true);
    log("[Simulation] eingeschaltet");
    server.send(200, "application/json", simulationJson());
}

void WebConsoleHandler::handleSimulationOff() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireSimulationAuthentication)) return;
    Simulation::RuntimeSimulation::setEnabled(false);
    log("[Simulation] ausgeschaltet");
    server.send(200, "application/json", simulationJson());
}

void WebConsoleHandler::handleSimulationToggle() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireSimulationAuthentication)) return;
    Simulation::RuntimeSimulation::toggle();
    log(String("[Simulation] ") + (Simulation::RuntimeSimulation::enabled() ? "eingeschaltet" : "ausgeschaltet"));
    server.send(200, "application/json", simulationJson());
}

void WebConsoleHandler::handleSimulationScenario() {
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
    log("[Simulation] Szenario: " + String(Simulation::RuntimeSimulation::scenarioName()));
    server.send(200, "application/json", simulationJson());
}

void WebConsoleHandler::handleUpdatePage() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireOtaAuthentication)) return;
    server.sendHeader("Location", "/#ota");
    server.send(302, "text/plain", "");
}

void WebConsoleHandler::handleUpdateFinished() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireOtaAuthentication)) return;
    const bool ok = !Update.hasError();
    if (!ok && webOtaStatus == "Bereit") {
        webOtaStatus = WebRuntimeHandlers::updateErrorText("Update fehlgeschlagen");
    }
    server.send(ok ? 200 : 500, "text/plain", ok ? "Update erfolgreich, Neustart..." : webOtaStatus);
    if (ok) {
        webOtaStatus = "Update erfolgreich, Neustart";
        log("[WebOTA] Update erfolgreich, Neustart");
        delay(500);
        ESP.restart();
    }
}

void WebConsoleHandler::handleUpdateUpload() {
    if (!WebSecurity::requireAuthentication(server, SecurityConfig::RequireOtaAuthentication)) return;
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        updateInProgress = true;
        if (SecurityConfig::RejectOtaWhenSketchSpaceUnknown && ESP.getFreeSketchSpace() == 0) {
            webOtaStatus = "OTA abgelehnt: freier Sketch-Speicher unbekannt";
            log("[WebOTA] " + webOtaStatus);
            return;
        }
        WebRuntimeHandlers::beginWebOtaUpload(upload.filename, webOtaStatus, webConsoleLogCallback);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        WebRuntimeHandlers::writeWebOtaChunk(upload.buf, upload.currentSize, webOtaStatus, webConsoleLogCallback);
    } else if (upload.status == UPLOAD_FILE_END) {
        WebRuntimeHandlers::finishWebOtaUpload(upload.totalSize, webOtaStatus, webConsoleLogCallback);
        updateInProgress = false;
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        WebRuntimeHandlers::abortWebOtaUpload(webOtaStatus, webConsoleLogCallback);
        updateInProgress = false;
    }
}
