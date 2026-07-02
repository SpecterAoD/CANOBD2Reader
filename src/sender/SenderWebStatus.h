#pragma once

#include <cstdint>

#include "WebRuntimeStatus.h"

namespace SenderWebStatus {

struct Input {
    bool canBusActive = false;
    bool canDriverReady = false;
    bool espNowReady = false;
    uint32_t lastCanMessageAt = 0;
    uint32_t heartbeatCount = 0;
};

Runtime::WebRuntimeStatus build(const Input& input);

} // namespace SenderWebStatus
