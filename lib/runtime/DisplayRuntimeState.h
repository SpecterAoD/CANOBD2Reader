#pragma once

#include <Arduino.h>
#include <cstdint>

namespace Runtime {

/// Volatile display-side receiver/runtime diagnostics.
///
/// Measurement values still live in DisplayData for rendering, but link health,
/// sequence tracking and receiver errors are runtime state and must not be
/// mixed with static configuration.
struct DisplayRuntimeState {
    uint32_t lastReceivedAt = 0;
    uint32_t receivedPackets = 0;
    uint32_t droppedPackets = 0;
    uint32_t crcErrors = 0;
    uint32_t lastSequence = 0;
    uint32_t lastHeartbeatAt = 0;
    uint32_t lastHeartbeatSequence = 0;
    uint32_t lastCanStatusAt = 0;
    uint32_t lastObdStatusAt = 0;
    String vehicleState = "Booting";
    String powerCommand = "None";
    uint8_t activityScore = 0;
    bool displaySleepRequested = false;
    uint32_t lastPowerStatusAt = 0;
    String lastRawPayload = "";
    String lastError = "Keine Daten";

    static DisplayRuntimeState& instance() {
        static DisplayRuntimeState state;
        return state;
    }
};

} // namespace Runtime
