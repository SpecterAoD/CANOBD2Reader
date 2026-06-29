#include "protocol.h"

namespace TelemetryProtocol {

void buildPayload(char* out,
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
