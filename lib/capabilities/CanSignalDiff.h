#pragma once

#include <cstddef>
#include <cstdint>

#include "CapabilityTypes.h"

namespace Capabilities {

struct CanFrameSample {
    uint32_t canId = 0;
    uint8_t length = 0;
    uint8_t data[8] = {};
};

std::size_t diffCanFrames(const CanFrameSample& before,
                          const CanFrameSample& after,
                          CanSignalCandidate* out,
                          std::size_t outCount);

uint8_t candidateConfidence(uint16_t changeCount);

} // namespace Capabilities
