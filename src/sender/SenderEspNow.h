#pragma once

namespace SenderEspNow {

/// Initializes the sender-side ESP-NOW peer used by the telemetry transport.
bool begin();

bool ready();
int meshId();

} // namespace SenderEspNow
