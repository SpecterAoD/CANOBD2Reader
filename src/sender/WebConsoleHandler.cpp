// Sender web console: device specific pages and APIs on top of the shared
// management server (lib/web/ManagementServer). Common endpoints such as
// signed firmware upload, GitHub updates, WiFi, logs, restart and simulation
// live in the shared module.
#include "WebConsoleHandler.h"

#include <WiFi.h>

#include "DeviceSecrets.h"
#include "DiagnosticLog.h"
#include "Logger.h"
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
.nav{display:grid;grid-auto-flow:column;grid-auto-columns:1fr;gap:7px;position:sticky;top:0;background:rgba(7,11,18,.92);backdrop-filter:blur(10px);padding:8px 0;z-index:2}
button,.btn{border:0;border-radius:14px;padding:11px 8px;background:#1f2937;color:var(--text);font-weight:700;font-size:13px;text-decoration:none;text-align:center}
button.active{background:var(--accent);color:#042f3d}.page{display:none}.page.active{display:block}.grid{display:grid;grid-template-columns:1fr 1fr;gap:9px}.card{background:var(--card);border:1px solid #263244;border-radius:18px;padding:12px;min-height:78px}.wide{grid-column:1/-1}.label{font-size:12px;color:var(--muted);margin-bottom:8px}.value{font-size:24px;font-weight:800;word-break:break-word}.small{font-size:13px;color:var(--muted);line-height:1.35}.okText{color:var(--ok)}.warnText{color:var(--warn)}.errText{color:var(--err)}
table{width:100%;border-collapse:collapse;font-size:12px}th,td{padding:7px 4px;border-bottom:1px solid #263244;text-align:left}th{color:var(--muted);font-weight:700}.status-OK{color:var(--ok)}.status-TIMEOUT,.status-PENDING{color:var(--warn)}.status-UNSUPPORTED{color:var(--muted)}.status-NEGATIVE_RESPONSE,.status-DECODE_ERROR,.status-SEND_FAILED{color:var(--err)}
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
<button data-page="cap">Cap</button>
<button data-page="log">Log</button>
<button data-page="ota">OTA</button>
<button data-page="gh">GitHub</button>
</nav>
<section id="dash" class="page active"><div class="grid">
<div class="card wide"><div class="label">Firmware</div><div class="small">Version: <span id="fw">--</span><br>Target: <span id="target">--</span><br>Protocol: <span id="proto">--</span><br>Build: <span id="build">--</span></div></div>
<div class="card"><div class="label">CAN</div><div id="can" class="value">--</div></div>
<div class="card"><div class="label">OBD2</div><div id="obd" class="value">--</div></div>
<div class="card"><div class="label">ESP-NOW</div><div id="espnow" class="value">--</div></div>
<div class="card"><div class="label">LED</div><div id="ledstate" class="value">--</div><div class="small">Test: <span id="ledtest">--</span><br>VehicleOff: <span id="vehicleoff">--</span><br>Wechsel: <span id="ledchange">--</span></div></div>
<div class="card wide"><div class="label">Power Manager</div><div id="vehiclestate" class="value">--</div><div class="small">Score: <span id="activityscore">--</span> · Command: <span id="powercmd">--</span><br>StartStop: <span id="startstop">--</span> · Parked: <span id="parked">--</span><br>Sleep Timer: <span id="sleeptimer">--</span></div></div>
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
<section id="cap" class="page"><div class="grid">
<div class="card wide"><div class="label">Capabilities</div><div id="capState" class="value">--</div><div class="small" id="capMsg">--</div><div class="actions"><button id="capObd" class="primary">OBD PID Scan starten</button><button id="capUds" class="primary">UDS Scan starten</button><button id="capCan">CAN Signal-Finder starten</button><button id="capCanBase">Baseline aufnehmen</button><button id="capCanAction">Aktion starten</button><button id="capCanFinish">Aktion stoppen / analysieren</button><button id="capStop" class="danger">Scan stoppen</button><a class="btn primary" href="/api/capabilities/export.json">Export JSON</a></div></div>
<div class="card wide"><div class="label">OBD PID Ergebnis</div><div class="small">Unterstuetzte PIDs werden testweise abgefragt. Nicht unterstuetzte PIDs bleiben grau, Timeouts orange/rot.</div><div style="overflow:auto"><table><thead><tr><th>PID</th><th>Name</th><th>Support</th><th>Antwort</th><th>Wert</th><th>Einheit</th><th>ms</th><th>Status</th></tr></thead><tbody id="capObdRows"></tbody></table></div></div>
<div class="card wide"><div class="label">UDS ECUs</div><div style="overflow:auto"><table><thead><tr><th>ECU</th><th>Antwort</th><th>Erreichbar</th><th>UDS</th><th>Name</th></tr></thead><tbody id="capEcuRows"></tbody></table></div></div>
<div class="card wide"><div class="label">UDS DID / ECU Infos</div><div style="overflow:auto"><table><thead><tr><th>ECU</th><th>DID</th><th>Name</th><th>Wert</th><th>Status</th></tr></thead><tbody id="capDidRows"></tbody></table></div></div>
<div class="card wide"><div class="label">CAN Signal-Finder Kandidaten</div><div class="small">Passiv ueber CanRouter: erst Baseline aufnehmen, dann Aktion starten, Ereignis im Fahrzeug ausloesen und danach analysieren.</div><div style="overflow:auto"><table><thead><tr><th>ID</th><th>Byte</th><th>Vorher</th><th>Nachher</th><th>Maske</th><th>Wechsel</th><th>%</th></tr></thead><tbody id="capCanRows"></tbody></table></div></div>
<div class="card wide"><div class="label">Rohdaten</div><pre id="capRaw">Noch kein Scan geladen.</pre></div>
</div></section>
<section id="log" class="page"><div class="grid">
<div class="card wide"><div class="label">Diagnose-Log</div><div class="small">Persistent: <span id="diaglog">--</span></div><div class="actions"><button id="loadLiveLog" class="primary">Live-Log laden</button><button id="loadFileLog" class="primary">Persistenten Log laden</button><a class="btn primary" href="/log/download">Log herunterladen</a><button id="clearLog" class="danger">Log löschen</button></div></div>
<div class="card wide"><div class="label">Live-Log</div><pre id="logs">Live-Log wird nicht automatisch geladen. Button drücken, damit das Diagnose-WLAN nicht unnötig belastet wird.</pre></div>
</div></section>
<section id="ota" class="page"><div class="card wide"><div class="label">Firmware über Web hochladen</div><div class="small">Es wird nur vom Projekt signierte Sender-Firmware akzeptiert. Unsignierte, veraenderte oder Display-Dateien werden abgelehnt, bevor umgeschaltet wird.</div></div>
<div class="card wide"><div class="label">OTA Status</div><div id="otastatus" class="value">--</div><div class="small">Frei: <span id="freeota">--</span> · Sketch: <span id="sketch">--</span> · Flash: <span id="flash">--</span></div></div>
<div class="actions"><a class="btn primary" href="/github-update">Signierte Firmware hochladen / GitHub-Update</a></div>
<div class="actions"><button id="restart" class="danger">Sender neu starten</button></div></section>
<section id="gh" class="page"><div class="grid">
<div class="card wide"><div class="label">GitHub Updates / Rollback</div><div class="small">Firmware per WLAN oder Handy-Hotspot aus dem GitHub-Manifest laden. Sender installiert nur Sender-Firmware. Rollback ist manuell und benötigt eine Browser-Bestätigung.</div><div class="actions"><a class="btn primary" href="/github-update">GitHub Update-Seite öffnen</a></div></div>
</div></section>
</div>
<script>
const $=id=>document.getElementById(id);
const scenarios=%%SIMULATION_SCENARIOS%%;
scenarios.forEach(x=>{let o=document.createElement('option');o.value=x;o.textContent=x;$('scenarioSelect').appendChild(o)});
document.querySelectorAll('.nav button').forEach(b=>b.onclick=()=>{document.querySelectorAll('.nav button').forEach(x=>x.classList.remove('active'));document.querySelectorAll('.page').forEach(x=>x.classList.remove('active'));b.classList.add('active');$(b.dataset.page).classList.add('active')});
function yn(v){return v?'aktiv':'aus'}function cls(el,state){el.className='value '+(state?'okText':'warnText')}function bytes(v){return Math.round((v||0)/1024)+' KB'}
async function apiGet(url){let r=await fetch(url,{cache:'no-store',credentials:'same-origin'});if(!r.ok)throw new Error(url+' HTTP '+r.status);return r}
async function apiPost(url){let r=await fetch(url,{method:'POST',credentials:'same-origin'});if(!r.ok)throw new Error(url+' HTTP '+r.status);return r}
function bind(id,fn){let el=$(id);if(el)el.onclick=fn}
async function refresh(){try{let s=await fetch('/status',{cache:'no-store'}).then(r=>r.json());$('ip').textContent=s.ip+' · '+Math.floor(s.uptime/1000)+'s';$('fw').textContent=s.firmware;$('target').textContent=s.target;$('proto').textContent=s.protocol;$('build').textContent=s.buildTime;$('otastatus').textContent=s.otaStatus;$('freeota').textContent=bytes(s.freeSketchSpace);$('sketch').textContent=bytes(s.sketchSize);$('flash').textContent=bytes(s.flashSize);$('diaglog').textContent=(s.diagnosticLogMounted?'bereit':'nicht gemountet')+' · '+bytes(s.diagnosticLogSize)+' / '+bytes(s.diagnosticLogMaxSize);$('run').textContent=s.started?'läuft':'gestoppt';$('run').className='pill '+(s.started?(s.securityReady?'ok':'warn'):'warn');$('can').textContent=s.canState;cls($('can'),s.canActive);$('obd').textContent=s.obdState||yn(s.obdActive);cls($('obd'),s.obdActive);$('espnow').textContent=s.espNowState;cls($('espnow'),s.espNowState==='READY');$('ledstate').textContent=s.ledState||'--';$('ledstate').className='value '+((s.ledState==='CanError'||s.ledState==='EspNowError')?'errText':(s.ledState==='VehicleOff'||s.ledState==='ObdTimeout'?'warnText':'okText'));$('ledtest').textContent=s.ledTestActive?'aktiv':'inaktiv';$('vehicleoff').textContent=s.vehicleOff?'ja':'nein';$('ledchange').textContent=s.ledLastStateChangeAt?((Math.max(0,s.uptime-s.ledLastStateChangeAt)/1000).toFixed(1)+'s'):'--';$('vehiclestate').textContent=s.vehicleState||'--';$('vehiclestate').className='value '+((s.vehicleState==='Parked'||s.vehicleState==='DisplaySleep')?'warnText':(s.vehicleState==='StartStop'?'warnText':'okText'));$('activityscore').textContent=s.activityScore;$('powercmd').textContent=s.powerCommand||'None';$('startstop').textContent=s.startStopDetected?'ja':'nein';$('parked').textContent=s.parkedDetected?'ja':'nein';$('sleeptimer').textContent=s.displaySleepDueAtMs?((Math.max(0,s.displaySleepDueAtMs-s.uptime)/1000).toFixed(0)+'s'):'--';$('heartbeat').textContent=s.heartbeats;$('bat').textContent=s.battery.toFixed(2)+' V';$('seq').textContent=s.seq;$('tel').textContent=s.lastTelemetry;$('pid').textContent=s.pidSupport?'bereit':'unbekannt';cls($('pid'),s.pidSupport);$('canage').textContent=(s.lastCanAge/1000).toFixed(1)+'s';$('dtc').textContent=s.lastDtc||'--';$('vin').textContent=s.lastVin||'--';$('obdReq').textContent=s.obdRequestCount;$('obdOk').textContent=s.obdValidResponseCount;$('obdTimeouts').textContent=s.obdTimeoutCount;$('obdNeg').textContent=s.obdNegativeResponseCount;$('obdSendFail').textContent=s.obdSendFailureCount;$('obdStreak').textContent=s.obdTimeoutStreak;$('obdReqId').textContent=s.obdRequestCanId;$('obdFallback').textContent=s.obdPhysicalFallbackActive?'0x7E0 aktiv':'0x7DF';$('lastObdReq').textContent=s.lastObdRequest||'--';$('lastEcuResp').textContent=s.lastEcuResponse||'--';$('lastNrc').textContent=s.lastNegativeResponse||'--';$('pidMask1').textContent=s.supportedPidMask01_20;$('pidMask2').textContent=s.supportedPidMask21_40;$('pidMask3').textContent=s.supportedPidMask41_60;$('udsAvail').textContent=s.udsAvailable?'ja':'nein';$('udsReq').textContent=s.udsRequestCount;$('udsOk').textContent=s.udsPositiveResponseCount;$('udsTimeouts').textContent=s.udsTimeoutCount;$('udsNeg').textContent=s.udsNegativeResponseCount;$('udsSendFail').textContent=s.udsSendFailureCount;$('lastUdsReq').textContent=s.lastUdsRequest||'--';$('lastUdsResp').textContent=s.lastUdsResponse||'--';$('udsDid').textContent=s.lastUdsDid||'--';$('udsDtc').textContent=s.lastUdsDtc||'--';$('udsNrc').textContent=s.lastUdsNegativeResponse||'--';$('err').textContent=s.securityWarning||s.lastSendError||s.lastError||'OK';}catch(e){$('run').textContent='offline';$('run').className='pill err'}}
async function refreshLog(){try{$('logs').textContent=await fetch('/log',{cache:'no-store'}).then(r=>r.text())}catch(e){}}
async function loadPersistentLog(){try{$('logs').textContent=await fetch('/log/file',{cache:'no-store'}).then(r=>r.text())}catch(e){}}
async function refreshSimulation(){try{let s=await fetch('/api/simulation',{cache:'no-store'}).then(r=>r.json());$('sim').textContent=s.simulation?'aktiv':'inaktiv';$('sim').className='value '+(s.simulation?'okText':'warnText');$('scenario').textContent=s.scenario||'--';$('scenarioSelect').value=s.scenario||'NormalSingleFrame'}catch(e){}}
async function loadSnapshot(){try{$('snapshot').textContent=JSON.stringify(await fetch('/api/diagnostic/snapshot',{cache:'no-store'}).then(r=>r.json()),null,2)}catch(e){$('snapshot').textContent='Snapshot konnte nicht geladen werden'}}
function esc(v){return String(v??'--').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
function cell(v,c=''){return '<td class="'+esc(c)+'">'+esc(v)+'</td>'}
function rowsObd(pids){return (pids||[]).map(p=>'<tr>'+cell(p.pid)+cell(p.name)+cell(p.supportedByMask?'ja':'nein')+cell(p.responded?'ja':'nein')+cell(p.exampleValue)+cell(p.unit)+cell(p.responseTimeMs)+cell(p.status,'status-'+p.status)+'</tr>').join('')||'<tr><td colspan="8">Noch keine OBD Ergebnisse</td></tr>'}
function rowsEcu(ecus){return (ecus||[]).map(e=>'<tr>'+cell(e.requestId)+cell(e.responseId)+cell(e.reachable?'ja':'nein')+cell(e.supportsUds?'ja':'nein')+cell(e.ecuName)+'</tr>').join('')||'<tr><td colspan="5">Noch keine UDS Ergebnisse</td></tr>'}
function rowsDid(dids){return (dids||[]).map(d=>'<tr>'+cell(d.ecu)+cell(d.did)+cell(d.name)+cell(d.value)+cell(d.status,'status-'+d.status)+'</tr>').join('')||'<tr><td colspan="5">Noch keine DID Ergebnisse</td></tr>'}
function rowsCan(cands){return (cands||[]).map(c=>'<tr>'+cell(c.canId)+cell(c.byteIndex)+cell(c.before)+cell(c.after)+cell(c.changedBitMask)+cell(c.changeCount)+cell(c.confidence)+'</tr>').join('')||'<tr><td colspan="7">Noch keine CAN Kandidaten</td></tr>'}
async function refreshCapabilities(){try{let c=await fetch('/api/capabilities/status',{cache:'no-store'}).then(r=>r.json());$('capState').textContent=c.scan+' / '+c.state;$('capState').className='value '+(c.active?'warnText':(c.state==='COMPLETED'?'okText':''));$('capMsg').textContent=(c.message||'--')+(c.canSniffer?' · Phase: '+c.canSniffer.phase+' · CAN Frames: '+c.canSniffer.frames+' · Kandidaten: '+c.canSniffer.candidateCount:'');$('capObdRows').innerHTML=rowsObd(c.obd&&c.obd.pids);$('capEcuRows').innerHTML=rowsEcu(c.uds&&c.uds.ecus);$('capDidRows').innerHTML=rowsDid(c.uds&&c.uds.dids);$('capCanRows').innerHTML=rowsCan(c.canSniffer&&c.canSniffer.candidates);$('capRaw').textContent=JSON.stringify(c,null,2)}catch(e){$('capState').textContent='offline';$('capState').className='value errText'}}
$('start').onclick=async()=>{await fetch('/start',{method:'POST'});refresh()}
$('restart').onclick=async()=>{if(confirm('Sender wirklich neu starten?'))await fetch('/api/restart',{method:'POST'})}
$('simToggle').onclick=async()=>{await fetch('/api/simulation/toggle',{method:'POST'});refreshSimulation()}
$('simOn').onclick=async()=>{await fetch('/api/simulation/on',{method:'POST'});refreshSimulation()}
$('simOff').onclick=async()=>{await fetch('/api/simulation/off',{method:'POST'});refreshSimulation()}
$('scenarioSelect').onchange=async()=>{await fetch('/api/simulation/scenario?scenario='+encodeURIComponent($('scenarioSelect').value),{method:'POST'});refreshSimulation()}
$('loadLiveLog').onclick=refreshLog
$('loadFileLog').onclick=loadPersistentLog
$('loadSnapshot').onclick=loadSnapshot
async function runCapabilityAction(url,label){$('capMsg').textContent=label+'...';try{await apiPost(url);await refreshCapabilities()}catch(e){$('capState').textContent='Fehler';$('capState').className='value errText';$('capMsg').textContent=e.message||'Aktion fehlgeschlagen'}}
bind('capObd',()=>runCapabilityAction('/api/capabilities/obd/start','OBD PID Scan startet'))
bind('capUds',()=>runCapabilityAction('/api/capabilities/uds/start','UDS Scan startet'))
bind('capCan',()=>runCapabilityAction('/api/capabilities/can/start','CAN Signal-Finder startet'))
bind('capCanBase',()=>runCapabilityAction('/api/capabilities/can/baseline','CAN Baseline wird aufgenommen'))
bind('capCanAction',()=>runCapabilityAction('/api/capabilities/can/action/start','CAN Aktion wird aufgezeichnet'))
bind('capCanFinish',()=>runCapabilityAction('/api/capabilities/can/action/finish','CAN Aktion wird analysiert'))
bind('capStop',()=>runCapabilityAction('/api/capabilities/stop','Scan wird gestoppt'))
$('clearLog').onclick=async()=>{if(confirm('Diagnose-Log wirklich löschen?')){await fetch('/api/log/clear',{method:'POST'});refreshLog();refresh()}}
setInterval(refresh,2500);setInterval(refreshSimulation,5000);setInterval(refreshCapabilities,5000);refresh();refreshSimulation();refreshCapabilities();
</script>
</body></html>
)rawliteral";
    String page(html);
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
