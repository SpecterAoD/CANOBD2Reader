#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "config/SenderConfig.h"
#include "config/NetworkConfig.h"
#include "Logger.h"

namespace OTAHandler {

    // Initialisiert OTA
    void initOTA();

    // Muss im Loop aufgerufen werden
    void handleOTA();
}
