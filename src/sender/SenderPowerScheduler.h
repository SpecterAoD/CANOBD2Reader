#pragma once

#include <cstdint>

namespace SenderPowerScheduler {

void reset();

/// Runs the non-blocking sender power maintenance tasks:
/// - updates vehicle/running state from battery voltage
/// - evaluates deep-sleep conditions
/// - publishes the periodic battery voltage telemetry
void tick(uint32_t nowMs);

} // namespace SenderPowerScheduler
