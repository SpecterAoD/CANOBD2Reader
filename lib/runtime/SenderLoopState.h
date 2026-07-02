#pragma once

#include <cstdint>

namespace Runtime {

struct SenderLoopState {
    uint32_t currentMillis = 0;
    uint32_t lastCanMessageAt = 0;
    uint32_t lastHeartbeatSentAt = 0;
    uint32_t lastTwaiStatusLogAt = 0;
    uint32_t heartbeatCount = 0;
    bool canBusActive = false;
    bool canDriverReady = false;
    bool espNowReady = false;

    void resetForBoot(uint32_t nowMs, uint32_t canIdleTimeoutMs) {
        currentMillis = nowMs;
        if (canIdleTimeoutMs > 5 && nowMs >= canIdleTimeoutMs - 5) {
            lastCanMessageAt = nowMs - canIdleTimeoutMs + 5;
        } else {
            lastCanMessageAt = 0;
        }
        lastHeartbeatSentAt = 0;
        lastTwaiStatusLogAt = 0;
        heartbeatCount = 0;
        canBusActive = false;
        canDriverReady = false;
        espNowReady = false;
    }

    void updateNow(uint32_t nowMs) {
        currentMillis = nowMs;
    }

    void markCanTraffic(uint32_t nowMs) {
        lastCanMessageAt = nowMs;
        canBusActive = true;
    }

    bool canRecent(uint32_t nowMs, uint32_t timeoutMs) const {
        return lastCanMessageAt > 0 && nowMs - lastCanMessageAt <= timeoutMs;
    }
};

} // namespace Runtime
