#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

namespace TelemetryProtocol {

constexpr size_t MaxPayloadLength = 160;
constexpr uint8_t Version = 2;

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
