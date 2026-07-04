#include "SenderLedController.h"

namespace Status {
namespace {
constexpr uint32_t kSlowBlinkPeriodMs = 1000;
constexpr uint32_t kFastBlinkPeriodMs = 250;
constexpr uint32_t kErrorPulseMs = 1000;
} // namespace

void SenderLedController::reset(uint32_t nowMs) {
    state_ = SenderLedState::Booting;
    stateBeforeLedTest_ = SenderLedState::Booting;
    lastStateChangeAt_ = nowMs;
    errorPulseUntilMs_ = 0;
    seenCanActivity_ = false;
    vehicleOff_ = false;
    ledTestWasActive_ = false;
}

void SenderLedController::requestErrorPulse(uint32_t nowMs) {
    errorPulseUntilMs_ = nowMs + kErrorPulseMs;
}

SenderLedOutput SenderLedController::update(const SenderLedInput& input) {
    if (input.canActive) {
        seenCanActivity_ = true;
    }

    vehicleOff_ = seenCanActivity_ && input.canDriverReady && !input.canActive;

    if (input.ledTestActive) {
        if (!ledTestWasActive_) {
            stateBeforeLedTest_ = state_;
        }
        ledTestWasActive_ = true;
        setState(SenderLedState::LedTest, input.nowMs);
        return {SenderLedState::LedTest, true, true, vehicleOff_};
    }

    if (ledTestWasActive_) {
        ledTestWasActive_ = false;
        state_ = stateBeforeLedTest_;
    }

    const SenderLedState baseState = chooseBaseState(input);
    setState(baseState, input.nowMs);

    SenderLedOutput output = outputForState(baseState, input.nowMs);
    output.vehicleOff = vehicleOff_;

    if (errorPulseUntilMs_ != 0 && input.nowMs < errorPulseUntilMs_) {
        output.errorOn = true;
    } else if (errorPulseUntilMs_ != 0 && input.nowMs >= errorPulseUntilMs_) {
        errorPulseUntilMs_ = 0;
    }

    return output;
}

const char* SenderLedController::stateName(SenderLedState state) {
    switch (state) {
        case SenderLedState::Booting: return "Booting";
        case SenderLedState::Ready: return "Ready";
        case SenderLedState::CanActive: return "CanActive";
        case SenderLedState::ObdActive: return "ObdActive";
        case SenderLedState::EspNowError: return "EspNowError";
        case SenderLedState::CanError: return "CanError";
        case SenderLedState::ObdTimeout: return "ObdTimeout";
        case SenderLedState::VehicleOff: return "VehicleOff";
        case SenderLedState::LedTest: return "LedTest";
    }
    return "Unknown";
}

bool SenderLedController::blink(uint32_t nowMs, uint32_t periodMs) {
    return ((nowMs / (periodMs / 2)) % 2U) == 0U;
}

SenderLedState SenderLedController::chooseBaseState(const SenderLedInput& input) const {
    if (!input.senderRunning) return SenderLedState::Booting;
    if (!input.espNowReady) return SenderLedState::EspNowError;
    if (input.canError || !input.canDriverReady) return SenderLedState::CanError;
    if (input.obdTimeout) return SenderLedState::ObdTimeout;
    if (vehicleOff_) return SenderLedState::VehicleOff;
    if (input.obdActive) return SenderLedState::ObdActive;
    if (input.canActive) return SenderLedState::CanActive;
    return SenderLedState::Ready;
}

SenderLedOutput SenderLedController::outputForState(SenderLedState state, uint32_t nowMs) const {
    switch (state) {
        case SenderLedState::Booting:
            return {state, blink(nowMs, kSlowBlinkPeriodMs), false, vehicleOff_};
        case SenderLedState::Ready:
        case SenderLedState::CanActive:
        case SenderLedState::ObdActive:
            return {state, true, false, vehicleOff_};
        case SenderLedState::VehicleOff:
            return {state, blink(nowMs, kSlowBlinkPeriodMs), false, vehicleOff_};
        case SenderLedState::ObdTimeout:
            return {state, blink(nowMs, kFastBlinkPeriodMs), false, vehicleOff_};
        case SenderLedState::EspNowError:
            return {state, false, blink(nowMs, kFastBlinkPeriodMs), vehicleOff_};
        case SenderLedState::CanError:
            return {state, false, true, vehicleOff_};
        case SenderLedState::LedTest:
            return {state, true, true, vehicleOff_};
    }
    return {SenderLedState::Booting, false, false, vehicleOff_};
}

void SenderLedController::setState(SenderLedState state, uint32_t nowMs) {
    if (state_ == state) return;
    state_ = state;
    lastStateChangeAt_ = nowMs;
}

} // namespace Status
