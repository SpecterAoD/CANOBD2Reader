#include "DisplayData.h"

#include "DiagnosticLog.h"
#include "StatusLogic.h"

namespace {
DisplayTelemetryValue statusValues[DisplayConfig::TelemetryValueCapacity];
uint8_t statusCount = 0;
DisplayLogic::LiveValueView liveView;
} // namespace

namespace DisplayData {

uint8_t currentPage = DisplayConfig::MainPageIndex;

Runtime::DisplayRuntimeState& runtime() {
    return Runtime::DisplayRuntimeState::instance();
}

DisplayLogic::LiveValueView& live() {
    return liveView;
}

DisplayTelemetryValue* findValue(const String& keyOrName) {
    for (uint8_t index = 0; index < statusCount; ++index) {
        if (statusValues[index].key == keyOrName || statusValues[index].name == keyOrName) return &statusValues[index];
    }
    return nullptr;
}

void upsertValue(const String& type, const String& key, const String& name, const String& value,
                 const String& unit, const String& status, uint32_t sequence) {
    DisplayTelemetryValue* entry = findValue(key);
    if (entry == nullptr) entry = findValue(name);
    if (entry == nullptr) {
        if (statusCount >= DisplayConfig::TelemetryValueCapacity) {
            runtime().lastError = "Statusliste voll";
            return;
        }
        entry = &statusValues[statusCount++];
    }
    entry->type = type;
    entry->key = key;
    entry->name = name;
    entry->value = value;
    entry->unit = unit;
    entry->status = status;
    entry->sequence = sequence;
    entry->updatedAt = millis();
}

bool isStatusFresh(const DisplayTelemetryValue* value) {
    return value != nullptr && millis() - value->updatedAt <= DisplayConfig::StatusTimeoutMs;
}

String displayText(const char* keyOrName) {
    const DisplayTelemetryValue* value = findValue(String(keyOrName));
    if (!isStatusFresh(value) || value->value.length() == 0 || value->value == "N/A") return "--";
    return value->value;
}

bool isEspNowConnected() {
    return StatusLogic::packetLinkHealth(millis(), runtime().lastReceivedAt, DisplayConfig::EspNowTimeoutMs) ==
           StatusLogic::Health::Ok;
}

bool isCanStatusRecent() {
    return runtime().lastCanStatusAt > 0 && millis() - runtime().lastCanStatusAt <= DisplayConfig::CanTimeoutMs;
}

bool isObdStatusRecent() {
    return runtime().lastObdStatusAt > 0 && millis() - runtime().lastObdStatusAt <= DisplayConfig::ObdTimeoutMs;
}

uint8_t visiblePageCount() {
    return runtime().diagnosticPagesEnabled ? DisplayConfig::PageCount : DisplayConfig::NormalPageCount;
}

void setDiagnosticPagesEnabled(bool enabled) {
    runtime().diagnosticPagesEnabled = enabled;
    if (currentPage >= visiblePageCount()) currentPage = DisplayConfig::MainPageIndex;
    DiagnosticLog::appendf("[DISPLAY] Diagnoseseiten %s", enabled ? "an" : "aus");
}

} // namespace DisplayData
