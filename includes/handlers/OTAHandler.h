#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "Config.h"
#include "Logger.h"

namespace OTAHandler {

    // Initialisiert OTA
    void initOTA();

    // Muss im Loop aufgerufen werden
    void handleOTA();
}