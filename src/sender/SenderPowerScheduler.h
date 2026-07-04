#pragma once

#include <cstdint>

#include "SenderLoopState.h"

namespace SenderPowerScheduler {

void reset();

/// Runs the non-blocking sender power maintenance tasks:
/// - updates vehicle/running state from battery voltage
/// - evaluates deep-sleep conditions
/// - publishes the periodic battery voltage telemetry
void tick(const Runtime::SenderLoopState& state, uint32_t lastObdResponseAt);

} // namespace SenderPowerScheduler
