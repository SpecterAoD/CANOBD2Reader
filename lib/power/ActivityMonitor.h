#pragma once

#include <cstdint>

namespace Power {

enum class VehicleState : uint8_t {
    Booting,
    Running,
    StartStop,
    Idle,
    Parked,
    DisplaySleep
};

enum class PowerCommand : uint8_t {
    None,
    Sleep,
    Wakeup
};

struct ActivityInput {
    uint32_t nowMs = 0;
    uint32_t bootCompletedAtMs = 0;
    uint32_t lastCanMessageAtMs = 0;
    uint32_t lastObdResponseAtMs = 0;
    uint32_t lastUserActivityAtMs = 0;
    float rpm = 0.0f;
    float speedKph = 0.0f;
    float batteryVoltage = 0.0f;
    float engineLoadPercent = 0.0f;
    float throttlePercent = 0.0f;
    bool simulationActive = false;
    bool canDriverReady = false;
    bool obdEnabled = true;
};

struct ActivitySnapshot {
    VehicleState state = VehicleState::Booting;
    PowerCommand command = PowerCommand::None;
    uint8_t activityScore = 0;
    bool canActive = false;
    bool obdActive = false;
    bool engineRunning = false;
    bool moving = false;
    bool boardAwake = false;
    bool userActive = false;
    bool startStopDetected = false;
    bool parkedDetected = false;
    uint32_t idleStartedAtMs = 0;
    uint32_t parkedStartedAtMs = 0;
    uint32_t displaySleepDueAtMs = 0;
    uint32_t lastWakeupAtMs = 0;
    uint32_t lastSleepAtMs = 0;
};

class ActivityMonitor {
public:
    void reset(uint32_t nowMs = 0);
    ActivitySnapshot update(const ActivityInput& input);
    const ActivitySnapshot& snapshot() const { return snapshot_; }

    static const char* vehicleStateName(VehicleState state);
    static const char* powerCommandName(PowerCommand command);

private:
    uint8_t calculateActivityScore(ActivitySnapshot& flags, const ActivityInput& input) const;
    VehicleState chooseState(const ActivityInput& input, const ActivitySnapshot& flags) const;
    PowerCommand chooseCommand(VehicleState previousState, VehicleState nextState, uint32_t nowMs);
    bool recently(uint32_t nowMs, uint32_t lastMs, uint32_t timeoutMs) const;

    ActivitySnapshot snapshot_{};
    VehicleState previousState_ = VehicleState::Booting;
};

} // namespace Power
