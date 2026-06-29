#include "WebConsoleHandler.h"

void WebConsoleHandler::begin() {
    if (!Config::EnableWebConsole) return;

    WiFi.mode(WIFI_AP);
    WiFi.softAP(Config::DebugAPSSID, Config::DebugAPPass);
    Serial.print("[WebConsole] AP gestartet: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/log", handleLog);
    server.on("/start", handleStart);
    server.begin();
}

void WebConsoleHandler::handle() {
    if (Config::EnableWebConsole) {
        server.handleClient();
    }
}

void WebConsoleHandler::log(const String& msg) {
    if (!Config::EnableWebConsole) {
        if (Config::SerialFlag) Serial.println(msg);
        return;
    }

    if (logBuffer.size() >= Config::WebConsoleMaxLines) {
        logBuffer.pop_front();
    }
    logBuffer.push_back(msg);

    if (Config::SerialFlag) {
        Serial.println(msg);
    }
}

void WebConsoleHandler::handleRoot() {
    String html =
        "<html><head><meta charset='UTF-8'></head><body>"
        "<h1>OBD Debug Console</h1>"
        "<button onclick=\"fetch('/start')\">Start</button>"
        "<pre id='console' style='background:#000;color:#0f0;height:80vh;overflow:auto'></pre>"
        "<script>"
        "setInterval(()=>fetch('/log').then(r=>r.text()).then(t=>console.innerText=t),1000);"
        "</script>"
        "</body></html>";

    server.send(200, "text/html", html);
}

void WebConsoleHandler::handleLog() {
    String page;
    for (auto &line : logBuffer) {
        page += line + "\n";
    }
    server.send(200, "text/plain", page);
}

void WebConsoleHandler::handleStart() {
    startRequested = true;
    server.send(200, "text/plain", "System gestartet");
}