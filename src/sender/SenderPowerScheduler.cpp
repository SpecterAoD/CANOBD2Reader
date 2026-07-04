#include "SenderPowerScheduler.h"

#include "SenderPower.h"
#include "config/SenderConfig.h"

namespace {
uint32_t lastBatterySendAt = 0;
}

namespace SenderPowerScheduler {

void reset() {
    lastBatterySendAt = 0;
}

void tick(const Runtime::SenderLoopState& state, uint32_t lastObdResponseAt) {
    const uint32_t nowMs = state.currentMillis;
    SenderPower::updateActivity(state, lastObdResponseAt);

    if (nowMs - lastBatterySendAt > SenderConfig::BatterySendIntervalMs) {
        SenderPower::sendBatteryVoltage();
        SenderPower::publishPowerTelemetry();
        lastBatterySendAt = nowMs;
    }
}

} // namespace SenderPowerScheduler
