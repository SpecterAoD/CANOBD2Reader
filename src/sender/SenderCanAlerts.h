#pragma once

#include <Arduino.h>

#include "SenderCallbacks.h"

namespace SenderCanAlerts {

struct Result {
    bool rxData = false;
    bool errorLedRequested = false;
    String errorText = "";
};

Result process(uint32_t waitMs, SenderCallbacks::SendStatus sendStatus);

} // namespace SenderCanAlerts
