#pragma once

#include <cstdint>

namespace SenderLedButton {

void begin();

/// Updates the non-blocking LED test button state.
bool updateLedTestButton();

/// Pulses the error LED without disturbing an active manual LED test.
void pulseError(uint32_t nowMs);

/// Turns the error LED off again after its pulse time elapsed.
void update(uint32_t nowMs);

bool ledTestActive();

} // namespace SenderLedButton
