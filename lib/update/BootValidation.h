#pragma once

// Automatic rollback of broken updates.
//
// After an OTA update the bootloader starts the new image in "pending verify"
// state. This module keeps it pending until the firmware ran StableAfterMs
// with successful local initialization and only then confirms it. If the new image crashes or hangs
// (watchdog reset) before that, the bootloader boots the previous firmware.

#include <cstdint>

namespace BootValidation {

constexpr uint32_t StableAfterMs = 30000;
constexpr uint32_t HealthDeadlineMs = 60000;

enum class Decision { Wait, Confirm, Rollback };

class HealthGate {
public:
    Decision evaluate(uint32_t nowMs, bool localHealthy) {
        if (!started_) { started_ = true; startedAt_ = nowMs; }
        if (!localHealthy) healthy_ = false;
        else if (!healthy_) { healthy_ = true; healthyAt_ = nowMs; }
        if (healthy_ && nowMs - healthyAt_ >= StableAfterMs) return Decision::Confirm;
        if (nowMs - startedAt_ >= HealthDeadlineMs) return Decision::Rollback;
        return Decision::Wait;
    }
private:
    uint32_t startedAt_ = 0;
    uint32_t healthyAt_ = 0;
    bool started_ = false;
    bool healthy_ = false;
};

// Call regularly from loop(); cheap after confirmation.
void tick(uint32_t nowMs, bool localHealthy);
// True if this boot is a freshly installed image waiting for confirmation.
bool pendingVerify();

} // namespace BootValidation
