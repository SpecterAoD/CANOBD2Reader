#pragma once

#include <Arduino.h>
#include <cstdint>

#include "SenderCallbacks.h"

namespace SenderUdsScheduler {

void reset();
void tick(uint32_t nowMs,
          uint32_t& lastCanMessageAt,
          uint32_t& lastObdResponseAt,
          bool& canBusActive,
          SenderCallbacks::SendTelemetry sendTelemetry,
          SenderCallbacks::SendStatus sendStatus);

const String& lastDidText();
const String& lastDtcText();

} // namespace SenderUdsScheduler
