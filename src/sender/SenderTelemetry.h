#pragma once

#include <Arduino.h>
#include <cstdint>

namespace SenderTelemetry {

void reset();

void send(const char* type,
          const char* key,
          const char* name,
          const char* value,
          const char* unit,
          const char* status);

void sendStatus(const char* key, const char* value, const char* status);

uint32_t sequence();
uint32_t sendOkCount();
uint32_t sendFailCount();
uint32_t lastObdResponseAt();
uint32_t& lastObdResponseAtRef();

const char* lastPayload();
const String& lastError();
const String& lastSendError();
void setLastError(const String& error);

} // namespace SenderTelemetry
