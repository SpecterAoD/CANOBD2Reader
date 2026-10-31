#pragma once

// Automatic rollback of broken updates.
//
// After an OTA update the bootloader starts the new image in "pending verify"
// state. This module keeps it pending until the firmware ran StableAfterMs
// without reset and only then confirms it. If the new image crashes or hangs
// (watchdog reset) before that, the bootloader boots the previous firmware.

#include <cstdint>

namespace BootValidation {

constexpr uint32_t StableAfterMs = 30000;

// Call regularly from loop(); cheap after confirmation.
void tick(uint32_t nowMs);
// True if this boot is a freshly installed image waiting for confirmation.
bool pendingVerify();

} // namespace BootValidation
