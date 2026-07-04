#pragma once

#include <cstdint>

#include "SenderLoopState.h"

namespace SenderLedButton {

void begin();

/// Updates the non-blocking LED test button state.
bool updateLedTestButton();

/// Pulses the error LED without disturbing an active manual LED test.
void pulseError(uint32_t nowMs);

/// Updates the physical LEDs from the current sender runtime state.
void update(const Runtime::SenderLoopState& state, uint32_t lastObdResponseAt);

bool ledTestActive();
bool vehicleOff();
const char* stateName();
uint32_t lastStateChangeAt();

} // namespace SenderLedButton
