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
            Serial.print("[DEBUG] ");
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

        Serial.print("[DEBUG] ");
        Serial.println(buf);
    }

    template<typename T>
    inline void debugValue(const char* label, T value, const char* unit = "") {
        if (Config::DebugFlag && Config::SerialFlag) {
            Serial.print("[DEBUG] ");
            Serial.print(label);
            Serial.print(": ");
            Serial.print(value);
            if (unit && unit[0] != '\0') {
                Serial.print(" ");
                Serial.print(unit);
            }
            Serial.println();
        }
    }

    // ======== Power Debug ========
    inline void power(const char* msg) {
        if (Config::PowerDebugFlag && Config::SerialFlag) {
            Serial.print("[POWER] ");
            Serial.println(msg);
        }
    }

    template<typename T>
    inline void powerValue(const char* label, T value, const char* unit = "") {
        if (Config::PowerDebugFlag && Config::SerialFlag) {
            Serial.print("[POWER] ");
            Serial.print(label);
            Serial.print(": ");
            Serial.print(value);
            if (unit && unit[0] != '\0') {
                Serial.print(" ");
                Serial.print(unit);
            }
            Serial.println();
        }
    }

    // ======== TWAI / CAN Debug ========
    inline void twai(const char* msg) {
        if (Config::TwaiDebugFlag && Config::SerialFlag) {
            Serial.print("[TWAI] ");
            Serial.println(msg);
        }
    }

    // ======== OBD Debug ========
    inline void obd(const char* msg) {
        if (Config::DebugFlag && Config::SerialFlag) {
            Serial.print("[OBD] ");
            Serial.println(msg);
        }
    }

    template<typename T>
    inline void obdValue(const char* label, T value, const char* unit = "") {
        if (Config::DebugFlag && Config::SerialFlag) {
            Serial.print("[OBD] ");
            Serial.print(label);
            Serial.print(": ");
            Serial.print(value);
            if (unit && unit[0] != '\0') {
                Serial.print(" ");
                Serial.print(unit);
            }
            Serial.println();
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