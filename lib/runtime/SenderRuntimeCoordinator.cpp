#include "SenderRuntimeCoordinator.h"

namespace Runtime {

SenderRuntimeCoordinator::SenderRuntimeCoordinator(const Config& config, const Services& services)
    : config_(config), services_(services) {}

void SenderRuntimeCoordinator::resetForBoot(uint32_t nowMs,
                                            uint32_t canIdleTimeoutMs,
                                            bool espNowReady,
                                            bool canDriverReady) {
    state_.resetForBoot(nowMs, canIdleTimeoutMs);
    state_.espNowReady = espNowReady;
    state_.canDriverReady = canDriverReady;
}

void SenderRuntimeCoordinator::tick(uint32_t nowMs) {
    state_.updateNow(nowMs);

    if (services_.handleOta != nullptr) services_.handleOta();
    if (services_.handleWeb != nullptr) services_.handleWeb();
    if (services_.updateWebStatus != nullptr) services_.updateWebStatus(state_);
    if (services_.updateLedTestButton != nullptr) services_.updateLedTestButton();
    if (services_.sendHeartbeat != nullptr) services_.sendHeartbeat(state_);

    if (services_.simulationEnabled != nullptr && services_.simulationEnabled()) {
        if (services_.tickSimulation != nullptr) services_.tickSimulation(state_.currentMillis);
        return;
    }

    if (services_.senderStarted != nullptr && !services_.senderStarted()) {
        return;
    }

    if (!state_.canDriverReady) {
        return;
    }

    if (services_.processCanAlerts != nullptr) {
        const CanAlertResult canAlerts = services_.processCanAlerts(config_.pollingRateMs);
        if (canAlerts.rxData) {
            state_.markCanTraffic(state_.currentMillis);
        }
        if (canAlerts.errorText != nullptr && canAlerts.errorText[0] != '\0' &&
            services_.setLastError != nullptr) {
            services_.setLastError(canAlerts.errorText);
        }
        if (canAlerts.errorLedRequested && services_.pulseErrorLed != nullptr) {
            services_.pulseErrorLed(state_.currentMillis);
        }
    }

    if (services_.updateLed != nullptr) services_.updateLed(state_.currentMillis);
    if (services_.tickObd != nullptr) services_.tickObd(state_);
    if (services_.tickUds != nullptr) services_.tickUds(state_);

    if (config_.twaiDebugEnabled &&
        state_.currentMillis - state_.lastTwaiStatusLogAt >= config_.twaiStatusLogIntervalMs) {
        if (services_.logTwaiStatus != nullptr) services_.logTwaiStatus();
        state_.lastTwaiStatusLogAt = state_.currentMillis;
    }

    if (services_.tickPower != nullptr) services_.tickPower(state_.currentMillis);
}

} // namespace Runtime
