#pragma once

#include <Arduino.h>
#include <cstdint>

#include "SenderCallbacks.h"

namespace SenderObdScheduler {

void reset();
void tick(uint32_t nowMs,
          uint32_t& lastCanMessageAt,
          uint32_t& lastObdResponseAt,
          bool& canBusActive,
          SenderCallbacks::SendTelemetry sendTelemetry,
          SenderCallbacks::SendStatus sendStatus);

bool supportedPidsInitialized();
const String& lastDtcText();
const String& lastVinText();

} // namespace SenderObdScheduler
