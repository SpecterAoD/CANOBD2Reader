#include <Arduino.h>

#include "core/Config.h"
#include "core/Logger.h"
#include "core/Utils.h"

#include "handlers/NetworkManager.h"
#include "handlers/OTAHandler.h"
#include "handlers/WebConsoleHandler.h"
#include "handlers/CANHandler.h"
#include "handlers/OBDHandler.h"
#include "handlers/PIDs.h"
#include "handlers/UpdateManager.h"
#include "handlers/PID_Converter.h"

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
    if (millis() - lastOBDRequest > Config::ObdRequestIntervalMs) {
        lastOBDRequest = millis();

        for (uint8_t pid : Config::ObdRequestedPids) {
            OBD2Handler::requestAndSendPID(pid);
            delay(50); // leichte Entzerrung
        }
    }
}