#pragma once
#include <Arduino.h>
#include "config/ProjectConfig.h"
#include "config/LoggingConfig.h"
#include <driver/twai.h>

namespace Logger {
    // Logging architecture:
    // - Logger::* writes category-based runtime logs to Serial when enabled.
    // - Logger::emit forwards the same line to an optional in-memory sink
    //   (for example WebConsole buffering via Logger::setSink).
    // - Persistent on-device diagnostics are written explicitly via
    //   DiagnosticLog::append/appendf in lib/logging.
    // - High-volume telemetry diagnostics use TraceSenderTelemetry /
    //   TraceDisplayTelemetry and must stay disabled unless actively debugging.
    using Sink = void (*)(const char*);
    inline Sink sink = nullptr;

    inline void setSink(Sink newSink) {
        sink = newSink;
    }

    inline void emit(const char* msg) {
        if (sink) sink(msg);
    }

    // ======== Startup ========
    inline void initDebug() {
        if (LoggingConfig::SerialEnabled) {
            Serial.begin(ProjectConfig::Baudrate);
            delay(100);  // Let USB serial settle before first output.
            Serial.println();
            Serial.print("Firmware: ");
            Serial.println(ProjectConfig::FirmwareVersion);
            Serial.print("Baudrate: ");
            Serial.println(ProjectConfig::Baudrate);

            if (LoggingConfig::GeneralDebugEnabled) Serial.println("[INFO] Debug aktiviert");
            if (LoggingConfig::TwaiDebugEnabled)    Serial.println("[INFO] TWAI Debug aktiviert");
            if (LoggingConfig::PowerDebugEnabled)   Serial.println("[INFO] Power Debug aktiviert");
            if (LoggingConfig::CanDebugEnabled)     Serial.println("[INFO] CAN Debug aktiviert");
            if (LoggingConfig::Obd2DebugEnabled)    Serial.println("[INFO] OBD2 Debug aktiviert");
        }
    }

    // ======== General debug ========
    inline void debug(const char* msg) {
        if (!LoggingConfig::GeneralDebugEnabled) return;
        emit(msg);
        if (LoggingConfig::SerialEnabled) {
            Serial.print("[DEBUG] ");
            Serial.println(msg);
        }
    }

    inline void debugf(const char* fmt, ...) {
        if (!LoggingConfig::GeneralDebugEnabled) return;

        char buf[128];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buf, sizeof(buf), fmt, args);
        va_end(args);

        emit(buf);
        if (LoggingConfig::SerialEnabled) {
            Serial.print("[DEBUG] ");
            Serial.println(buf);
        }
    }

    template<typename T>
    inline void debugValue(const char* label, T value, const char* unit = "") {
        if (!LoggingConfig::GeneralDebugEnabled) return;

        const String valueText = String(value);
        const bool hasUnit = unit && unit[0] != '\0';
        char buf[128];
        snprintf(buf, sizeof(buf), "%s: %s%s%s", label, valueText.c_str(), hasUnit ? " " : "", hasUnit ? unit : "");
        emit(buf);

        if (LoggingConfig::SerialEnabled) {
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

    // ======== Power debug ========
    inline void power(const char* msg) {
        if (!LoggingConfig::PowerDebugEnabled) return;
        emit(msg);
        if (LoggingConfig::SerialEnabled) {
            Serial.print("[POWER] ");
            Serial.println(msg);
        }
    }

    template<typename T>
    inline void powerValue(const char* label, T value, const char* unit = "") {
        if (!LoggingConfig::PowerDebugEnabled) return;

        const String valueText = String(value);
        const bool hasUnit = unit && unit[0] != '\0';
        char buf[128];
        snprintf(buf, sizeof(buf), "%s: %s%s%s", label, valueText.c_str(), hasUnit ? " " : "", hasUnit ? unit : "");
        emit(buf);

        if (LoggingConfig::SerialEnabled) {
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

    // ======== TWAI / CAN debug ========
    inline void twai(const char* msg) {
        if (!LoggingConfig::TwaiDebugEnabled) return;
        emit(msg);
        if (LoggingConfig::SerialEnabled) {
            Serial.print("[TWAI] ");
            Serial.println(msg);
        }
    }

    inline void can(const char* msg) {
        if (!LoggingConfig::CanDebugEnabled) return;
        emit(msg);
        if (LoggingConfig::SerialEnabled) {
            Serial.print("[CAN] ");
            Serial.println(msg);
        }
    }

    inline void canFrame(const twai_message_t& message) {
        if (!LoggingConfig::CanDebugEnabled) return;

        char buf[160];
        int used = snprintf(buf, sizeof(buf), "[CAN] ID=0x%lX DLC=%u Data=",
                            static_cast<unsigned long>(message.identifier),
                            static_cast<unsigned>(message.data_length_code));
        for (int i = 0; i < message.data_length_code && used > 0 && used < static_cast<int>(sizeof(buf)); i++) {
            used += snprintf(buf + used, sizeof(buf) - used, " %02X", message.data[i]);
        }
        emit(buf);

        if (LoggingConfig::SerialEnabled) {
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

    // ======== OBD2 debug (high-volume, keep disabled by default) ========
    inline void obd(const char* msg) {
        if (!LoggingConfig::Obd2DebugEnabled) return;
        emit(msg);
        if (LoggingConfig::SerialEnabled) {
            Serial.print("[OBD2] ");
            Serial.println(msg);
        }
    }

    inline void obdFrame(uint8_t pid, const uint8_t* data, uint8_t len) {
        if (!LoggingConfig::Obd2DebugEnabled) return;

        char buf[128];
        int used = snprintf(buf, sizeof(buf), "[OBD2] PID 0x%02X Data=", pid);
        for (uint8_t i = 0; i < len && used > 0 && used < static_cast<int>(sizeof(buf)); i++) {
            used += snprintf(buf + used, sizeof(buf) - used, " %02X", data[i]);
        }
        emit(buf);

        if (LoggingConfig::SerialEnabled) {
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
        if (!LoggingConfig::Obd2DebugEnabled) return;

        const String valueText = String(value);
        const bool hasUnit = unit && unit[0] != '\0';
        char buf[128];
        snprintf(buf, sizeof(buf), "[OBD2] %s: %s%s%s",
                 label,
                 valueText.c_str(),
                 hasUnit ? " " : "",
                 hasUnit ? unit : "");
        emit(buf);

        if (LoggingConfig::SerialEnabled) {
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
    // ======== Warning level (always routed to sink, serial optional) ========
    inline void warn(const char* msg) {
        emit(msg);
        if (LoggingConfig::SerialEnabled) {
            Serial.print("[WARN] ");
            Serial.println(msg);
        }
        // Optional: escalate through an external channel if needed.
    }
    // ======== Alarm level (always routed to sink, serial optional) ========
    inline void alarm(const char* msg) {
        emit(msg);
        if (LoggingConfig::SerialEnabled) {
            Serial.print("[ALARM] ");
            Serial.println(msg);
        }
        // Optional: escalate through an external channel if needed.
    }
    // ======== Critical level (always routed to sink, serial optional) ========
    inline void critical(const char* msg) {
        emit(msg);
        if (LoggingConfig::SerialEnabled) {
            Serial.print("[CRITICAL] ");
            Serial.println(msg);
        }
        // Optional: escalate through an external channel if needed.
    }
}
