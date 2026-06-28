#pragma once

#include <Arduino.h>

namespace TelemetryProtocol {

constexpr size_t MaxPayloadLength = 128;
constexpr uint8_t Version = 2;

// Gemeinsamer ESP-NOW-Textframe fuer Sender und Anzeige.
// Der CRC wird ueber den kompletten Frame ohne das letzte crc-Feld berechnet.
struct TextFrame {
    char payload[MaxPayloadLength]{};
    uint16_t crc = 0;
};

inline uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

inline void finalizeFrame(TextFrame& frame) {
    frame.payload[MaxPayloadLength - 1] = '\0';
    frame.crc = crc16(reinterpret_cast<const uint8_t*>(&frame), sizeof(TextFrame) - sizeof(frame.crc));
}

inline bool validateFrame(const TextFrame& frame) {
    return crc16(reinterpret_cast<const uint8_t*>(&frame), sizeof(TextFrame) - sizeof(frame.crc)) == frame.crc;
}

inline void buildPayload(char* out,
                         size_t outSize,
                         const char* type,
                         const char* key,
                         const char* name,
                         const char* value,
                         const char* unit,
                         const char* status,
                         uint32_t sequence) {
    snprintf(out, outSize, "%s,%s,%s,%s,%s,%s,%lu",
             type, key, name, value, unit, status,
             static_cast<unsigned long>(sequence));
}

} // namespace TelemetryProtocol
