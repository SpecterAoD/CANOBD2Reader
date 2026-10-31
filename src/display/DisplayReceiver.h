#pragma once

#include <cstdint>

// ESP-NOW reception of sender telemetry (live snapshots and status texts).
namespace DisplayReceiver {
void begin();
void processQueuedPackets();
uint32_t macFilteredCount();
uint32_t queueDropCount();
}
