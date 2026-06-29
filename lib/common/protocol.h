#pragma once

#include <Arduino.h>
#include "shared_types.h"

namespace TelemetryProtocol {

constexpr size_t MaxPayloadLength = CommonTypes::MaxPayloadLength;
constexpr uint8_t Version = 2;

using TextFrame = CommonTypes::EspNowTextFrame;

uint16_t crc16(const uint8_t* data, size_t length);
void finalizeFrame(TextFrame& frame);
bool validateFrame(const TextFrame& frame);

void buildPayload(char* out,
                  size_t outSize,
                  const char* type,
                  const char* key,
                  const char* name,
                  const char* value,
                  const char* unit,
                  const char* status,
                  uint32_t sequence);

} // namespace TelemetryProtocol
