#pragma once
#include <Arduino.h>
#include "Config.h"
#include <driver/twai.h>

namespace Logger {
    using Sink = void (*)(const char*);
    inline Sink sink = nullptr;

    inline void setSink(Sink newSink) {
        sink = newSink;
    }

    inline void emit(const char* msg) {
        if (sink) sink(msg);
    }

    // ======== Initialisierung ========
    inline void initDebug() {
        if (Config::Debug::Serial) {
            Serial.begin(Config::Project::Baudrate);
            delay(100);  // USB stabilisieren
            Serial.println();
            Serial.print("Firmware: ");
            Serial.println(Config::Project::FirmwareVersion);
            Serial.print("Baudrate: ");
            Serial.println(Config::Project::Baudrate);

            if (Config::Debug::General) Serial.println("[INFO] Debug aktiviert");
            if (Config::Debug::Twai)    Serial.println("[INFO] TWAI Debug aktiviert");
            if (Config::Debug::Power)   Serial.println("[INFO] Power Debug aktiviert");
            if (Config::Debug::Can)     Serial.println("[INFO] CAN Debug aktiviert");
            if (Config::Debug::Obd2)    Serial.println("[INFO] OBD2 Debug aktiviert");
        }
    }

    // ======== Allgemeines Debug ========
    inline void debug(const char* msg) {
        emit(msg);
        if (Config::Debug::General && Config::Debug::Serial) {
            Serial.print("[DEBUG] ");
            Serial.println(msg);
        }
    }

    inline void debugf(const char* fmt, ...) {
        if (!(Config::Debug::General && Config::Debug::Serial)) return;

        char buf[128];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);

        Serial.print("[DEBUG] ");
        Serial.println(buf);
        emit(buf);
    }

    template<typename T>
    inline void debugValue(const char* label, T value, const char* unit = "") {
        if (Config::Debug::General && Config::Debug::Serial) {
            Serial.print("[DEBUG] ");
            Serial.print(label);
            Serial.print(": ");
            Serial.print(value);
            if (unit && unit[0] != '\0') {
                Serial.print(" ");
                Serial.print(unit);
            }
            Serial.println();
            char buf[128];
            snprintf(buf, sizeof(buf), "%s: %s%s", label, String(value).c_str(), unit && unit[0] ? unit : "");
            emit(buf);
        }
    }

    // ======== Power Debug ========
    inline void power(const char* msg) {
        emit(msg);
        if (Config::Debug::Power && Config::Debug::Serial) {
            Serial.print("[POWER] ");
            Serial.println(msg);
        }
    }

    template<typename T>
    inline void powerValue(const char* label, T value, const char* unit = "") {
        if (Config::Debug::Power && Config::Debug::Serial) {
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
        emit(msg);
        if (Config::Debug::Twai && Config::Debug::Serial) {
            Serial.print("[TWAI] ");
            Serial.println(msg);
        }
    }

    inline void can(const char* msg) {
        emit(msg);
        if (Config::Debug::Can && Config::Debug::Serial) {
            Serial.print("[CAN] ");
            Serial.println(msg);
        }
    }

    inline void canFrame(const twai_message_t& message) {
        if (Config::Debug::Can && Config::Debug::Serial) {
            Serial.print("[CAN] ID=0x");
            Serial.print(message.identifier, HEX);
            Serial.print(" DLC=");
            Serial.print(message.data_length_code);
            Serial.print(" Data=");
            for (int i = 0; i < message.data_length_code; i++) {
                Serial.print(" ");
                if (message.data[i] < 0x10) Serial.print("0");
                Serial.print(message.data[i], HEX);
            }
            Serial.println();
        }
    }

    // ======== OBD2 Debug ========
    inline void obd(const char* msg) {
        emit(msg);
        if (Config::Debug::Obd2 && Config::Debug::Serial) {
            Serial.print("[OBD2] ");
            Serial.println(msg);
        }
    }

    inline void obdFrame(uint8_t pid, const uint8_t* data, uint8_t len) {
        if (Config::Debug::Obd2 && Config::Debug::Serial) {
            Serial.print("[OBD2] PID 0x");
            Serial.print(pid, HEX);
            Serial.print(" Data=");
            for (uint8_t i = 0; i < len; i++) {
                Serial.print(" ");
                if (data[i] < 0x10) Serial.print("0");
                Serial.print(data[i], HEX);
            }
            Serial.println();
        }
    }

    template<typename T>
    inline void obdValue(const char* label, T value, const char* unit = "") {
        if (Config::Debug::Obd2 && Config::Debug::Serial) {
            Serial.print("[OBD2] ");
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
    // ======== Warn Debug (immer wenn Serial aktive) ========
    inline void warn(const char* msg) {
        emit(msg);
        if (Config::Debug::Serial) {
            Serial.print(" [WARN] ");
            Serial.println(msg);
        }
        // Optional: senden per ESP-NOW
    }
    // ======== Alarm Debug (immer wenn Serial aktiv) ========
    inline void alarm(const char* msg) {
        emit(msg);
        if (Config::Debug::Serial) {
            Serial.print("[ALARM] ");
            Serial.println(msg);
        }
        // Optional: hier könnte man noch ESP-NOW Versand ergänzen
    }
    // ======== criticl Debug (immer wenn Serial aktiv) ========
    inline void critical(const char* msg) {
        emit(msg);
        if (Config::Debug::Serial) {
            Serial.print("[CRITICAL] ");
            Serial.println(msg);
        }
        // Optional: senden per ESP-NOW
    }
}
