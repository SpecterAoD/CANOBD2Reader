#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include "CapabilityTypes.h"

namespace SenderCapabilityScanner {

enum class ActiveScan : uint8_t {
    Idle,
    ObdPids,
    Uds,
    CanSniffer
};

void reset();
bool active();
ActiveScan activeScan();

void startObdPidScan();
void startUdsScan();
void startCanSniffer();
void resetCanSnifferBaseline();
void stop();

/// Runs at most one scanner step. Normal OBD/UDS polling is paused while this is active.
void tick(uint32_t nowMs);

String statusJson();
String exportJson();

} // namespace SenderCapabilityScanner
