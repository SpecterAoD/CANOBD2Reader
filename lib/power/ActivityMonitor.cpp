#include "ActivityMonitor.h"

#include "config/DisplayConfig.h"
#include "config/PowerConfig.h"

namespace Power {

void ActivityMonitor::reset(uint32_t nowMs) {
    snapshot_ = ActivitySnapshot{};
    snapshot_.state = VehicleState::Booting;
    snapshot_.idleStartedAtMs = 0;
    snapshot_.parkedStartedAtMs = 0;
    snapshot_.displaySleepDueAtMs = 0;
    previousState_ = VehicleState::Booting;
    snapshot_.lastWakeupAtMs = nowMs;
}

ActivitySnapshot ActivityMonitor::update(const ActivityInput& input) {
    ActivitySnapshot next = snapshot_;
    next.command = PowerCommand::None;

    next.activityScore = calculateActivityScore(next, input);
    const VehicleState selected = chooseState(input, next);

    if (selected == VehicleState::Idle && snapshot_.state != VehicleState::Idle) {
        next.idleStartedAtMs = input.nowMs;
    }

    if (selected == VehicleState::Parked && snapshot_.state != VehicleState::Parked) {
        next.parkedStartedAtMs = input.nowMs;
        next.displaySleepDueAtMs = input.nowMs + PowerConfig::DisplaySleepAfterMs;
    }

    if (selected != VehicleState::Parked && selected != VehicleState::DisplaySleep) {
        next.parkedStartedAtMs = 0;
        next.displaySleepDueAtMs = 0;
    }

    VehicleState finalState = selected;
    if (selected == VehicleState::Parked &&
        next.displaySleepDueAtMs != 0 &&
        input.nowMs >= next.displaySleepDueAtMs) {
        finalState = VehicleState::DisplaySleep;
    }

    next.command = chooseCommand(snapshot_.state, finalState, input.nowMs);
    next.state = finalState;
    next.startStopDetected = finalState == VehicleState::StartStop;
    next.parkedDetected = finalState == VehicleState::Parked || finalState == VehicleState::DisplaySleep;

    previousState_ = snapshot_.state;
    snapshot_ = next;
    return snapshot_;
}

const char* ActivityMonitor::vehicleStateName(VehicleState state) {
    switch (state) {
        case VehicleState::Booting: return "Booting";
        case VehicleState::Running: return "Running";
        case VehicleState::StartStop: return "StartStop";
        case VehicleState::Idle: return "Idle";
        case VehicleState::Parked: return "Parked";
        case VehicleState::DisplaySleep: return "DisplaySleep";
    }
    return "Unknown";
}

const char* ActivityMonitor::powerCommandName(PowerCommand command) {
    switch (command) {
        case PowerCommand::None: return "None";
        case PowerCommand::Sleep: return "Sleep";
        case PowerCommand::Wakeup: return "Wakeup";
    }
    return "None";
}

uint8_t ActivityMonitor::calculateActivityScore(ActivitySnapshot& flags, const ActivityInput& input) const {
    flags.canActive = recently(input.nowMs, input.lastCanMessageAtMs, DisplayConfig::CanTimeoutMs);
    flags.obdActive = recently(input.nowMs, input.lastObdResponseAtMs, DisplayConfig::ObdTimeoutMs);
    flags.engineRunning = input.rpm >= PowerConfig::EngineRunningRpm;
    flags.moving = input.speedKph >= PowerConfig::MovingSpeedKph;
    flags.boardAwake = input.batteryVoltage >= PowerConfig::BoardVoltageAwakeMin;
    flags.userActive = recently(input.nowMs, input.lastUserActivityAtMs, DisplayConfig::ValueTimeoutMs);

    uint8_t score = 0;
    if (flags.canActive) score += 5;
    if (flags.obdActive) score += 5;
    if (flags.engineRunning) score += 3;
    if (flags.moving) score += 3;
    if (input.engineLoadPercent >= PowerConfig::ActiveEngineLoadPercent) score += 2;
    if (input.throttlePercent >= PowerConfig::ActiveThrottlePercent) score += 2;
    if (flags.boardAwake) score += 2;
    if (flags.userActive) score += 1;
    if (input.simulationActive) score += 5;
    return score;
}

VehicleState ActivityMonitor::chooseState(const ActivityInput& input, const ActivitySnapshot& flags) const {
    if (!PowerConfig::EnablePowerManager) return VehicleState::Running;

    const bool booting = input.bootCompletedAtMs == 0 || input.nowMs - input.bootCompletedAtMs < 1500;
    if (booting) return VehicleState::Booting;

    if (flags.engineRunning || flags.moving) return VehicleState::Running;

    const bool noBusActivity = !flags.canActive && !flags.obdActive;
    const bool parkTimeoutElapsed =
        noBusActivity &&
        input.lastCanMessageAtMs > 0 &&
        input.lastObdResponseAtMs > 0 &&
        input.nowMs - input.lastCanMessageAtMs >= PowerConfig::ParkDetectionTimeoutMs &&
        input.nowMs - input.lastObdResponseAtMs >= PowerConfig::ParkDetectionTimeoutMs;

    if (parkTimeoutElapsed && !input.simulationActive) return VehicleState::Parked;

    if (PowerConfig::EnableStartStopDetection &&
        !flags.engineRunning &&
        !flags.moving &&
        flags.canActive &&
        flags.obdActive &&
        flags.boardAwake) {
        return VehicleState::StartStop;
    }

    if (flags.canActive || flags.obdActive || flags.boardAwake || flags.userActive) {
        return VehicleState::Idle;
    }

    if (flags.activityScore == 0) return VehicleState::Parked;
    return VehicleState::Idle;
}

PowerCommand ActivityMonitor::chooseCommand(VehicleState previousState, VehicleState nextState, uint32_t nowMs) {
    if (nextState == VehicleState::DisplaySleep && previousState != VehicleState::DisplaySleep) {
        snapshot_.lastSleepAtMs = nowMs;
        return PowerCommand::Sleep;
    }

    if (previousState == VehicleState::DisplaySleep && nextState != VehicleState::DisplaySleep) {
        snapshot_.lastWakeupAtMs = nowMs;
        return PowerCommand::Wakeup;
    }

    if ((previousState == VehicleState::Parked || previousState == VehicleState::DisplaySleep) &&
        (nextState == VehicleState::Running || nextState == VehicleState::StartStop || nextState == VehicleState::Idle)) {
        snapshot_.lastWakeupAtMs = nowMs;
        return PowerCommand::Wakeup;
    }

    return PowerCommand::None;
}

bool ActivityMonitor::recently(uint32_t nowMs, uint32_t lastMs, uint32_t timeoutMs) const {
    return lastMs > 0 && nowMs >= lastMs && nowMs - lastMs <= timeoutMs;
}

} // namespace Power
