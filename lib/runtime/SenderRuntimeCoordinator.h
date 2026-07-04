#pragma once

#include <cstdint>

#include "SenderLoopState.h"

namespace Runtime {

class SenderRuntimeCoordinator {
public:
    struct Config {
        uint32_t pollingRateMs = 0;
        uint32_t twaiStatusLogIntervalMs = 0;
        bool twaiDebugEnabled = false;
    };

    struct CanAlertResult {
        bool rxData = false;
        bool errorLedRequested = false;
        const char* errorText = nullptr;
    };

    struct Services {
        void (*handleOta)() = nullptr;
        void (*handleWeb)() = nullptr;
        void (*updateWebStatus)(const SenderLoopState& state) = nullptr;
        void (*updateLedTestButton)() = nullptr;
        void (*sendHeartbeat)(SenderLoopState& state) = nullptr;
        bool (*simulationEnabled)() = nullptr;
        void (*tickSimulation)(uint32_t nowMs) = nullptr;
        bool (*senderStarted)() = nullptr;
        CanAlertResult (*processCanAlerts)(uint32_t waitMs) = nullptr;
        void (*setLastError)(const char* errorText) = nullptr;
        void (*pulseErrorLed)(uint32_t nowMs) = nullptr;
        void (*updateLed)(const SenderLoopState& state) = nullptr;
        void (*tickObd)(SenderLoopState& state) = nullptr;
        void (*tickUds)(SenderLoopState& state) = nullptr;
        void (*logTwaiStatus)() = nullptr;
        void (*tickPower)(const SenderLoopState& state) = nullptr;
    };

    SenderRuntimeCoordinator(const Config& config, const Services& services);

    void resetForBoot(uint32_t nowMs, uint32_t canIdleTimeoutMs, bool espNowReady, bool canDriverReady);
    void tick(uint32_t nowMs);

    SenderLoopState& state() { return state_; }
    const SenderLoopState& state() const { return state_; }

private:
    Config config_;
    Services services_;
    SenderLoopState state_{};
};

} // namespace Runtime
