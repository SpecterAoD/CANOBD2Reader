#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include "Config.h"
#include "Logger.h"

namespace UpdateManager {

    void initOTA();                       // Initialisierung von OTA
    void handleOTA();                     // Muss in loop() aufgerufen werden

    bool connectHotspot();                // Verbindung zu Hotspot
    bool checkForUpdate();                // Prüft, ob eine neue Version verfügbar ist
    bool downloadAndUpdate();             // Führt Update durch, wenn neue Firmware vorhanden
}