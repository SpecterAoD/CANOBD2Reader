#pragma once

#include "DisplayTypes.h"

namespace DisplayData {
  extern DisplayTelemetryValue values[34];
  extern uint8_t valueCount;
  extern uint8_t currentPage;
  extern uint8_t lastRenderedPage;
  extern uint32_t lastScreenRefresh;
  extern uint32_t lastReceivedAt;
  extern uint32_t lastButtonAt;
  extern uint32_t receivedPackets;
  extern uint32_t droppedPackets;
  extern uint32_t crcErrors;
  extern uint32_t lastSequence;
  extern uint32_t lastHeartbeatAt;
  extern uint32_t lastHeartbeatSequence;
  extern uint32_t lastCanStatusAt;
  extern uint32_t lastObdStatusAt;
  extern String lastRawPayload;
  extern String lastError;
  extern uint32_t lastInternalSimulationUpdate;
  extern size_t internalSimulationIndex;
  extern bool renderDirty;

  DisplayTelemetryValue* findValue(const String& name);
  DisplayTelemetryValue* upsertValue(const String& type,
                                     const String& key,
                                     const String& name,
                                     const String& value,
                                     const String& unit,
                                     const String& status,
                                     uint32_t sequence);
  bool isConnected();
  bool isEspNowConnected();
  bool isCanStatusRecent();
  bool isObdStatusRecent();
  bool isFresh(const DisplayTelemetryValue* value);
  DisplayLogic::DisplaySeverity severityForValue(const char* name);
  uint16_t colorForSeverity(DisplayLogic::DisplaySeverity severity);
  String displayValue(const char* name, uint8_t decimals = 0);
  String displayText(const char* name);
  uint16_t valueColor(const char* name);
  void markDirty();
}
