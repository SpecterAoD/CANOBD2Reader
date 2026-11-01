#pragma once

// Single ESP-NOW send path of the sender. Text packets carry status and
// diagnostic messages ("TYPE,KEY,NAME,VALUE,UNIT,STATUS"), binary packets the
// live snapshot. Every packet gets the next sequence number.

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include "TelemetryPacket.h"

namespace SenderTelemetry {

void reset();

void send(const char* type,
          const char* key,
          const char* name,
          const char* value,
          const char* unit,
          const char* status);

void sendStatus(const char* key, const char* value, const char* status);

// Binary payload (e.g. LiveData snapshot); shares sequence and statistics.
void sendBinary(Telemetry::PacketType type, const uint8_t* data, std::size_t length);

// Delivery result from the ESP-NOW send callback (any task).
void onDelivery(bool delivered);
// True while the display does not acknowledge (sending is reduced to 1/s).
bool linkOffline();

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
