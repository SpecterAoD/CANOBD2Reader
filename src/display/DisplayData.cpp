#include "DisplayData.h"
#include "StatusLogic.h"

namespace {
  float g_filteredSpeed = 0.0f;
  float g_filteredRpm = 0.0f;
  bool g_speedInitialized = false;
  bool g_rpmInitialized = false;

  bool isSpeedMetric(const String& key, const String& name) {
    return key == "0D" || key == "Speed" || name == "Speed" || name == "VehicleSpeed";
  }

  bool isRpmMetric(const String& key, const String& name) {
    return key == "0C" || key == "RPM" || name == "RPM" || name == "EngineRPM";
  }

  String smoothNumericValue(const String& key, const String& name, const String& inputValue) {
    String numericRaw = inputValue;
    numericRaw.replace(",", ".");
    const float parsed = numericRaw.toFloat();

    if (isSpeedMetric(key, name)) {
      if (!g_speedInitialized) {
        g_filteredSpeed = parsed;
        g_speedInitialized = true;
      } else {
        g_filteredSpeed = g_filteredSpeed + (parsed - g_filteredSpeed) * DisplayConfig::SpeedSmoothingAlpha;
      }
      return String(g_filteredSpeed, 1);
    }

    if (isRpmMetric(key, name)) {
      if (parsed >= DisplayConfig::RpmWarn) {
        return inputValue;
      }
      if (!g_rpmInitialized) {
        g_filteredRpm = parsed;
        g_rpmInitialized = true;
      } else {
        g_filteredRpm = g_filteredRpm + (parsed - g_filteredRpm) * DisplayConfig::RpmSmoothingAlpha;
      }
      return String(g_filteredRpm, 1);
    }

    return inputValue;
  }

  DisplayTelemetryValue* findAlias(const char* a, const char* b = nullptr, const char* c = nullptr) {
    using namespace DisplayData;
    DisplayTelemetryValue* value = nullptr;
    if (a != nullptr) value = findValue(String(a));
    if (value == nullptr && b != nullptr) value = findValue(String(b));
    if (value == nullptr && c != nullptr) value = findValue(String(c));
    return value;
  }

  DisplayTelemetryValue* resolveMetric(const char* name) {
    using namespace DisplayData;
    if (name == nullptr) return nullptr;

    String n(name);
    if (n == "Speed") return findAlias("Speed", "VehicleSpeed", "0D");
    if (n == "RPM") return findAlias("RPM", "EngineRPM", "0C");
    if (n == "CoolantTemp") return findAlias("CoolantTemp", "EngineCoolantTemp", "05");
    if (n == "BatteryVoltage") return findAlias("BatteryVoltage", "ControlVoltage", "VOLTAGE");
    if (n == "OilTemp") return findAlias("OilTemp", "EngineOilTemp", "5C");
    if (n == "EngineLoad") return findAlias("EngineLoad", "Load", "04");
    if (n == "IntakeTemp") return findAlias("IntakeTemp", "IntakeAirTemp", "0F");
    if (n == "AverageConsumption") return findAlias("AverageConsumption", "AVG");
    if (n == "InstantConsumption") return findAlias("InstantConsumption", "INST", "Consumption");
    if (n == "FuelRate") return findAlias("FuelRate", "EngineFuelRate", "5E");
    if (n == "Throttle") return findAlias("Throttle", "ThrottlePosition", "11");
    if (n == "MAF") return findAlias("MAF", "MassAirFlow", "10");
    if (n == "FuelLevel") return findAlias("FuelLevel", "2F");
    if (n == "RunTime") return findAlias("RunTime", "Runtime", "1F");
    if (n == "AmbientTemp") return findAlias("AmbientTemp", "46");
    if (n == "ManifoldAbsolutePressure") return findAlias("ManifoldAbsolutePressure", "MAP", "0B");
    if (n == "BarometricPressure") return findAlias("BarometricPressure", "Baro", "33");
    if (n == "BoostPressureBar") return findAlias("BoostPressureBar", "Boost", "BOOST");
    if (n == "UDS_DTC") return findAlias("UDS_DTC", "UDS");
    if (n == "UDS_NRC") return findAlias("UDS_NRC", "UDS_NEGATIVE", "UDS");
    if (n == "ReachableEcus") return findAlias("ReachableEcus", "ECU_COUNT");
    if (n == "UdsPending") return findAlias("UdsPending", "UDS_PENDING");
    if (n == "UdsBackoff") return findAlias("UdsBackoff", "UDS_BACKOFF");
    if (n == "CanSniffer") return findAlias("CanSniffer", "CAN_SNIFFER");
    if (n == "CanBaseline") return findAlias("CanBaseline", "CAN_BASELINE");
    if (n == "CanCandidates") return findAlias("CanCandidates", "CAN_CANDIDATES");
    if (n == "CanCandidateId") return findAlias("CanCandidateId", "CAN_CANDIDATE_ID");
    if (n == "PowerState") return findAlias("PowerState", "POWER_STATE");
    if (n == "PowerCommand") return findAlias("PowerCommand", "POWER_COMMAND");
    if (n == "ActivityScore") return findAlias("ActivityScore", "ACTIVITY_SCORE");
    return findAlias(name);
  }

  bool hasNumericContent(const String& text) {
    for (size_t i = 0; i < text.length(); ++i) {
      const char ch = text[i];
      if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+') return true;
    }
    return false;
  }
}

namespace DisplayData {
  DisplayTelemetryValue values[34];
  uint8_t valueCount = 0;
  uint8_t currentPage = 0;
  uint8_t lastRenderedPage = 255;
  uint32_t lastScreenRefresh = 0;
  uint32_t lastButtonAt = 0;
  uint32_t lastInternalSimulationUpdate = 0;
  size_t internalSimulationIndex = 0;
  bool renderDirty = true;

  Runtime::DisplayRuntimeState& runtime() {
    return Runtime::DisplayRuntimeState::instance();
  }

  void markDirty() {
    renderDirty = true;
  }

  DisplayTelemetryValue* findValue(const String& name) {
    for (uint8_t i = 0; i < valueCount; i++) {
      if (values[i].name == name || values[i].key == name) return &values[i];
    }
    return nullptr;
  }

  DisplayTelemetryValue* upsertValue(const String& type,
                                     const String& key,
                                     const String& name,
                                     const String& value,
                                     const String& unit,
                                     const String& status,
                                     uint32_t sequence) {
    DisplayTelemetryValue* existing = findValue(name);
    if (existing == nullptr) existing = findValue(key);

    if (existing == nullptr) {
      if (valueCount >= (sizeof(values) / sizeof(values[0]))) {
        runtime().droppedPackets++;
        runtime().lastError = "Werteliste voll";
        return nullptr;
      }
      existing = &values[valueCount++];
    }

    const String smoothedValue = smoothNumericValue(key, name, value);

    existing->type = type;
    existing->key = key;
    existing->name = name;
    existing->value = smoothedValue;
    existing->unit = unit;
    existing->status = status;
    existing->sequence = sequence;
    existing->updatedAt = millis();
    renderDirty = true;
    return existing;
  }

  bool isConnected() {
    return isEspNowConnected();
  }

  bool isEspNowConnected() {
    return StatusLogic::packetLinkHealth(millis(), runtime().lastReceivedAt, DisplayConfig::EspNowTimeoutMs) == StatusLogic::Health::Ok;
  }

  bool isCanStatusRecent() {
    return runtime().lastCanStatusAt > 0 && millis() - runtime().lastCanStatusAt <= DisplayConfig::CanTimeoutMs;
  }

  bool isObdStatusRecent() {
    return runtime().lastObdStatusAt > 0 && millis() - runtime().lastObdStatusAt <= DisplayConfig::ObdTimeoutMs;
  }

  bool isFresh(const DisplayTelemetryValue* value) {
    if (value == nullptr || millis() - value->updatedAt > DisplayConfig::ValueTimeoutMs) return false;
    String status = value->status;
    status.toUpperCase();
    if (status == "TIMEOUT" || status == "UNSUPPORTED" || status == "ERROR" || status == "SEND_FAIL") return false;
    return true;
  }

  String displayValue(const char* name, uint8_t decimals) {
    DisplayTelemetryValue* value = resolveMetric(name);
    if (!isFresh(value)) return "--";
    if (value->value == "N/A" || value->value.length() == 0) return "--";

    String raw = value->value;
    raw.trim();

    String result;
    if (hasNumericContent(raw)) {
      String numericRaw = raw;
      numericRaw.replace(",", ".");
      result = String(numericRaw.toFloat(), static_cast<unsigned int>(decimals));
    } else {
      result = raw;
    }

    if (value->unit.length() > 0 && result.indexOf(value->unit) < 0) result += " " + value->unit;
    return result;
  }

  String displayText(const char* name) {
    DisplayTelemetryValue* value = resolveMetric(name);
    if (value == nullptr || millis() - value->updatedAt > DisplayConfig::ValueTimeoutMs) return "--";
    if (value->value.length() == 0 || value->value == "N/A") return "--";
    return value->value;
  }

  DisplayLogic::DisplaySeverity severityForValue(const char* name) {
    DisplayTelemetryValue* value = resolveMetric(name);
    if (value == nullptr) {
      return DisplayLogic::DisplaySeverity::Unknown;
    }

    const bool fresh = isFresh(value);
    return DisplayLogic::severityForMetric(name, value->value.toFloat(), fresh, value->status.c_str());
  }

  uint16_t colorForSeverity(DisplayLogic::DisplaySeverity severity) {
    switch (severity) {
      case DisplayLogic::DisplaySeverity::Ok: return DisplayConfig::Ok;
      case DisplayLogic::DisplaySeverity::Warning: return DisplayConfig::Warn;
      case DisplayLogic::DisplaySeverity::Critical: return DisplayConfig::Error;
      case DisplayLogic::DisplaySeverity::Timeout: return DisplayConfig::Muted;
      case DisplayLogic::DisplaySeverity::Unknown: return DisplayConfig::Muted;
    }
    return DisplayConfig::Muted;
  }

  uint16_t valueColor(const char* name) {
    return colorForSeverity(severityForValue(name));
  }
}
