#pragma once

#include <Arduino.h>

namespace CommonTypes {

constexpr size_t MaxPayloadLength = 128;

struct EspNowTextFrame {
    char payload[MaxPayloadLength]{};
    uint16_t crc = 0;
};

struct EspNowCanFrame {
    uint8_t magic = 0x42;
    uint32_t timestamp = 0;
    int meshId = 0;
    uint32_t canId = 0;
    uint8_t len = 0;
    uint8_t data[8]{};
    uint16_t crc = 0;
};

struct TelemetryValue {
    const char* type;
    const char* key;
    const char* name;
    const char* unit;
};

} // namespace CommonTypes
