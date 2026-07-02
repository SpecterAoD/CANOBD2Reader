#pragma once

#include <cstdint>

namespace SenderSimulationScheduler {

void reset();
void tick(uint32_t nowMs);

} // namespace SenderSimulationScheduler
