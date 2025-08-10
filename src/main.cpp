#include <Arduino.h>

#include "Config.h"
#include "Logger.h"
#include "Utils.h"

#include "NetworkManager.h"
#include "OTAHandler.h"
#include "WebConsoleHandler.h"
#include "CANHandler.h"
#include "OBDHandler.h"
#include "PIDs.h"
#include "UpdateManager.h"
#include "PID_Converter.h"

void setup() {
   // ---------------- Initialisierung ----------------
    Logger::initDebug();

    // --------- Optional: WebConsole Mode ----------
    if (Config::EnableWebConsole) {
        Logger::debug("Starte WebConsole-Modus...");
        NetworkManager::startAccessPoint();      // Eigenes WLAN starten
        WebConsoleHandler::begin();              // Webserver starten

        Logger::debug("Warte auf START-Button...");
        while (!WebConsoleHandler::startRequested()) {
            WebConsoleHandler::loop();           // Webserver aktiv halten
            delay(100);
        }
        Logger::debug("WebConsole: Start bestätigt!");
    } else {
        // Optional
    }
    // ------ init ------
    NetworkManager::initESPNow();
    OTAHandler::initOTA();
    CANHandler::init();

}
void loop() {
// --------- OTA prüfen ----------
    OTAHandler::loop();

    // --------- WebConsole aktualisieren ----------
    if (Config::EnableWebConsole) {
        WebConsoleHandler::loop();
    }

    // --------- CAN-Nachrichten empfangen ----------
    CANHandler::processIncoming(); // Empfängt & sendet an ESP-NOW

    // --------- OBD2-PIDs regelmäßig abfragen ----------
    if (millis() - Config::lastObdRequestTime > Config::ObdRequestIntervalMs) {
        Config::lastObdRequestTime = millis();

        for (uint8_t pid : Config::ObdRequestedPids) {
            OBD2Handler::requestAndSendPID(pid);
            delay(50); // leichte Entzerrung
        }
    }
}