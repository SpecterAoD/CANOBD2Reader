#pragma once
#include <Arduino.h>
#include "Config.h"

namespace Logger {

    // ======== Initialisierung ========
    inline void initDebug() {
        if (Config::SerialFlag) {
            Serial.begin(Config::Baudrate);
            delay(100);  // USB stabilisieren
            Serial.println();
            Serial.print("Firmware: ");
            Serial.println(Config::FirmwareVersion);
            Serial.print("Baudrate: ");
            Serial.println(Config::Baudrate);

            if (Config::DebugFlag)      Serial.println("[INFO] Debug aktiviert");
            if (Config::TwaiDebugFlag)  Serial.println("[INFO] TWAI Debug aktiviert");
            if (Config::PowerDebugFlag) Serial.println("[INFO] Power Debug aktiviert");
        }
    }

    // ======== Allgemeines Debug ========
    inline void debug(const char* msg) {
        if (Config::DebugFlag && Config::SerialFlag) {
            Serial.println(msg);
        }
    }

    inline void debugf(const char* fmt, ...) {
        if (!(Config::DebugFlag && Config::SerialFlag)) return;

        char buf[128];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);

        Serial.println(buf);
    }

    // ======== Power Debug ========
    inline void power(const char* msg) {
        if (Config::PowerDebugFlag && Config::SerialFlag) {
            Serial.print("[POWER] ");
            Serial.println(msg);
        }
    }

    // ======== TWAI / CAN Debug ========
    inline void twai(const char* msg) {
        if (Config::TwaiDebugFlag && Config::SerialFlag) {
            Serial.print("[TWAI] ");
            Serial.println(msg);
        }
    }

    // ======== Alarm Debug (immer wenn Serial aktiv) ========
    inline void alarm(const char* msg) {
        if (Config::SerialFlag) {
            Serial.print("[ALARM] ");
            Serial.println(msg);
        }
        // Optional: hier könnte man noch ESP-NOW Versand ergänzen
    }
}