#pragma once

namespace SenderCallbacks {

using SendTelemetry = void (*)(const char* type,
                               const char* key,
                               const char* name,
                               const char* value,
                               const char* unit,
                               const char* status);

using SendStatus = void (*)(const char* key, const char* value, const char* status);

} // namespace SenderCallbacks
