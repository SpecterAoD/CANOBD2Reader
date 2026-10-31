#pragma once

// Display data model:
//   * live()      typed live values from the binary snapshot (protocol v3)
//   * status table text values from status/diagnostic packets (CAN, OBD, DTC,
//                 VIN, UDS, power, CAN hints) keyed by KEY or NAME
//   * runtime()   link statistics and power state

#include "DisplayTypes.h"
#include "DisplayRuntimeState.h"
#include "LiveValueView.h"

namespace DisplayData {

extern uint8_t currentPage;

Runtime::DisplayRuntimeState& runtime();
DisplayLogic::LiveValueView& live();

DisplayTelemetryValue* findValue(const String& keyOrName);
void upsertValue(const String& type, const String& key, const String& name, const String& value,
                 const String& unit, const String& status, uint32_t sequence);

// Text of a status value, "--" when unknown or older than StatusTimeoutMs.
String displayText(const char* keyOrName);
bool isStatusFresh(const DisplayTelemetryValue* value);

bool isEspNowConnected();
bool isCanStatusRecent();
bool isObdStatusRecent();

uint8_t visiblePageCount();
void setDiagnosticPagesEnabled(bool enabled);

} // namespace DisplayData
