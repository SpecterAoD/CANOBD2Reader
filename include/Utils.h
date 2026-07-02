#pragma once

#include <Arduino.h>
#include <esp_now.h>

#include "config/ProjectConfig.h"
#include "TelemetryCodec.h"
#include "TelemetryProtocol.h"
#include "TelemetrySequence.h"
#include "EspNowTelemetryTransport.h"

namespace Utils {

inline Telemetry::PacketType packetTypeFor(const char* type) {
    if (type == nullptr) return Telemetry::PacketType::Text;
    if (strcmp(type, "OBD") == 0) return Telemetry::PacketType::Obd;
    if (strcmp(type, "CAN") == 0) return Telemetry::PacketType::Can;
    if (strcmp(type, "DTC") == 0) return Telemetry::PacketType::Diagnostic;
    if (strcmp(type, "STATUS") == 0) return Telemetry::PacketType::Status;
    return Telemetry::PacketType::Text;
}

inline void sendTelemetryPacket(const char* payload, Telemetry::PacketType type = Telemetry::PacketType::Text) {
    const uint32_t sequence = Telemetry::nextSequence();
    Telemetry::TelemetryPacket packet{};
    Telemetry::TelemetryCodec::encodeText(packet, type, sequence, millis(), payload);
    const esp_err_t result = Transport::sendTelemetryPacket(packet);
    Transport::logTelemetrySendResult(result, sequence, payload);
}

inline void sendTelemetry(const char* type,
                          const char* key,
                          const char* name,
                          const char* value,
                          const char* unit,
                          const char* status) {
    const uint32_t sequence = Telemetry::nextSequence();
    char payload[ProjectConfig::TelemetryPayloadSize];
    TelemetryProtocol::buildPayload(payload, sizeof(payload),
                                    type, key, name, value, unit, status, sequence);
    Telemetry::TelemetryPacket packet{};
    Telemetry::TelemetryCodec::encodeText(packet, packetTypeFor(type), sequence, millis(), payload);
    const esp_err_t result = Transport::sendTelemetryPacket(packet);
    Transport::logTelemetrySendResult(result, sequence, payload);
}

inline void sendCanFrame(uint32_t canId, const uint8_t* data, uint8_t len) {
    char payload[96];
    size_t offset = snprintf(payload, sizeof(payload), "CAN,RAW,LastCAN,0x%03lX DLC%u",
                             static_cast<unsigned long>(canId),
                             static_cast<unsigned int>(len > 8 ? 8 : len));
    for (uint8_t i = 0; i < len && i < 8 && offset < sizeof(payload); ++i) {
        offset += snprintf(payload + offset, sizeof(payload) - offset, " %02X", data[i]);
    }
    snprintf(payload + offset, sizeof(payload) - offset, ",,OK");
    sendTelemetryPacket(payload, Telemetry::PacketType::Can);
}

inline void sendRawOBDData(byte pid, const uint8_t* data, byte len) {
    char buffer[64];
    size_t offset = snprintf(buffer, sizeof(buffer), "%02X,", pid);
    for (byte i = 0; i < len && offset < sizeof(buffer); ++i) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%02X ", data[i]);
    }
    sendTelemetryPacket(buffer, Telemetry::PacketType::Obd);
}

inline void sendOBDValue(byte pid, const char* name, float value, const char* unit) {
    char pidHex[4];
    char valueText[16];
    snprintf(pidHex, sizeof(pidHex), "%02X", pid);
    snprintf(valueText, sizeof(valueText), "%.2f", static_cast<double>(value));
    sendTelemetry("OBD", pidHex, name, valueText, unit, "OK");
}

inline void sendOBDError(byte pid, const char* name) {
    char pidHex[4];
    snprintf(pidHex, sizeof(pidHex), "%02X", pid);
    sendTelemetry("OBD", pidHex, name, "N/A", "", "TIMEOUT");
}

}
