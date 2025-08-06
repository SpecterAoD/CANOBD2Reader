#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "core/Config.h"
#include "core/Logger.h"

namespace OTAHandler {

    // Initialisiert OTA
    void initOTA();

    // Muss im Loop aufgerufen werden
    void handleOTA();
}