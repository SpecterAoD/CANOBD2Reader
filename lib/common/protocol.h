#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

namespace TelemetryProtocol {

// Text payload of status/diagnostic packets:
//   "TYPE,KEY,NAME,VALUE,UNIT,STATUS,SEQ"
// Live values are binary (lib/telemetry/LiveData.h).

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
