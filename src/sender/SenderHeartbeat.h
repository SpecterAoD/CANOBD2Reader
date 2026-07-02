#pragma once

#include <cstdint>

#include "SenderCallbacks.h"

namespace SenderHeartbeat {

struct Input {
    bool espNowReady = false;
    bool senderRunning = false;
    bool canDriverReady = false;
    bool canRecent = false;
    bool obdEnabled = false;
    bool obdRecent = false;
    bool udsEnabled = false;
    bool udsAvailable = false;
    bool simulationEnabled = false;
};

bool tick(uint32_t nowMs,
          uint32_t& lastSentAt,
          uint32_t& heartbeatCount,
          const Input& input,
          SenderCallbacks::SendStatus sendStatus);

} // namespace SenderHeartbeat
