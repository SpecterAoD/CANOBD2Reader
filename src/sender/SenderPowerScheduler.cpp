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

void tick(uint32_t nowMs) {
    SenderPower::updateCarStatus();
    SenderPower::handleSleep();

    if (nowMs - lastBatterySendAt > SenderConfig::BatterySendIntervalMs) {
        SenderPower::sendBatteryVoltage();
        lastBatterySendAt = nowMs;
    }
}

} // namespace SenderPowerScheduler
