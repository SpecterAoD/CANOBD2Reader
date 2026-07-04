#pragma once

#include <cstdint>

namespace Status {

/// Sender LED states are intentionally coarse-grained.
///
/// The green LED primarily means "the sender firmware is alive". CAN/OBD
/// details are represented by state names, blink patterns and the second/error
/// LED so the device does not look dead when the vehicle is temporarily off.
enum class SenderLedState : uint8_t {
    Booting,
    Ready,
    CanActive,
    ObdActive,
    EspNowError,
    CanError,
    ObdTimeout,
    VehicleOff,
    LedTest
};

struct SenderLedInput {
    uint32_t nowMs = 0;
    bool senderRunning = false;
    bool canDriverReady = false;
    bool canActive = false;
    bool obdActive = false;
    bool espNowReady = false;
    bool canError = false;
    bool obdTimeout = false;
    bool ledTestActive = false;
};

struct SenderLedOutput {
    SenderLedState state = SenderLedState::Booting;
    bool greenOn = false;
    bool errorOn = false;
    bool vehicleOff = false;
};

class SenderLedController {
public:
    void reset(uint32_t nowMs = 0);
    void requestErrorPulse(uint32_t nowMs);
    SenderLedOutput update(const SenderLedInput& input);

    SenderLedState state() const { return state_; }
    const char* stateName() const { return stateName(state_); }
    uint32_t lastStateChangeAt() const { return lastStateChangeAt_; }
    bool vehicleOff() const { return vehicleOff_; }
    bool hasSeenCanActivity() const { return seenCanActivity_; }

    static const char* stateName(SenderLedState state);

private:
    static bool blink(uint32_t nowMs, uint32_t periodMs);
    SenderLedState chooseBaseState(const SenderLedInput& input) const;
    SenderLedOutput outputForState(SenderLedState state, uint32_t nowMs) const;
    void setState(SenderLedState state, uint32_t nowMs);

    SenderLedState state_ = SenderLedState::Booting;
    SenderLedState stateBeforeLedTest_ = SenderLedState::Booting;
    uint32_t lastStateChangeAt_ = 0;
    uint32_t errorPulseUntilMs_ = 0;
    bool seenCanActivity_ = false;
    bool vehicleOff_ = false;
    bool ledTestWasActive_ = false;
};

} // namespace Status
