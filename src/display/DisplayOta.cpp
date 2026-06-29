#include "DisplayOta.h"
#include "DisplayData.h"
#include "Config.h"
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>

namespace {
  WebServer server(Config::Network::WebServerPort);
  bool webUpdateInProgress = false;

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

  void handleRoot() {
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
<div class="card"><div class="label">Status</div><div id="status" class="value">--</div><div id="detail" class="small">--</div></div>
<button class="danger" id="restart">Display neu starten</button>
</div>
<script>
const $=id=>document.getElementById(id);
async function refresh(){try{let s=await fetch('/status',{cache:'no-store'}).then(r=>r.json());$('ip').textContent=s.ip+' · '+Math.floor(s.uptime/1000)+'s';$('status').textContent=s.update?'Update läuft':'Bereit';$('detail').textContent=s.lastError||'Keine Fehler';$('state').textContent=s.update?'Update':'OK';$('state').className='pill '+(s.update?'err':'ok')}catch(e){$('state').textContent='offline';$('state').className='pill err'}}
$('restart').onclick=async()=>{if(confirm('Display wirklich neu starten?'))await fetch('/restart',{method:'POST'})}
setInterval(refresh,1000);refresh();
</script>
</body></html>
)rawliteral";
    server.send_P(200, "text/html", html);
  }

  void handleStatus() {
    String json;
    json.reserve(220);
    json += "{";
    json += "\"ip\":\"" + jsonEscape(WiFi.softAPIP().toString()) + "\",";
    json += "\"uptime\":" + String(millis()) + ",";
    json += "\"update\":" + String(webUpdateInProgress ? "true" : "false") + ",";
    json += "\"lastError\":\"" + jsonEscape(DisplayData::lastError) + "\"";
    json += "}";
    server.send(200, "application/json", json);
  }

  void handleRestart() {
    server.send(200, "text/plain", "Neustart");
    delay(250);
    ESP.restart();
  }

  void handleUpdateFinished() {
    const bool ok = !Update.hasError();
    server.send(200, "text/plain", ok ? "Update erfolgreich, Neustart..." : "Update fehlgeschlagen");
    DisplayData::lastError = ok ? "Web-OTA erfolgreich" : "Web-OTA fehlgeschlagen";
    if (ok) {
      delay(500);
      ESP.restart();
    }
  }

  void handleUpdateUpload() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      webUpdateInProgress = true;
      DisplayData::lastError = "Web-OTA Upload gestartet";
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        DisplayData::lastError = "Update.begin fehlgeschlagen";
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        DisplayData::lastError = "Web-OTA Schreibfehler";
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        DisplayData::lastError = "Web-OTA abgeschlossen";
      } else {
        DisplayData::lastError = "Update.end fehlgeschlagen";
      }
      webUpdateInProgress = false;
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      Update.end();
      webUpdateInProgress = false;
      DisplayData::lastError = "Web-OTA abgebrochen";
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
