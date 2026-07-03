#pragma once

#include <cstdint>

#include "CapabilityTypes.h"

namespace Capabilities {

struct UdsServiceProbe {
    uint8_t service;
    const char* name;
};

struct UdsDidProbe {
    uint16_t did;
    const char* name;
};

constexpr uint32_t UdsFirstRequestId = 0x7E0;
constexpr uint32_t UdsLastRequestId = 0x7E7;

bool isUdsResponsePending(uint8_t nrc);
bool isUdsUnsupportedNrc(uint8_t nrc);
CapabilityStatus statusForNegativeResponse(uint8_t nrc);
bool shouldContinueWaitingForPending(uint32_t startMs, uint32_t nowMs, uint32_t totalTimeoutMs);
uint32_t udsResponseIdForRequestId(uint32_t requestId);
bool isUdsRequestIdInScanRange(uint32_t requestId);

const UdsServiceProbe* udsServiceProbes(uint8_t& count);
const UdsDidProbe* udsDidProbes(uint8_t& count);

} // namespace Capabilities
