#include "DisplaySeverity.h"

#include <cstring>
#include "config/DisplayConfig.h"

namespace {

bool equalsAny(const char* value, const char* a, const char* b = nullptr, const char* c = nullptr) {
    if (value == nullptr) return false;
    return (a != nullptr && std::strcmp(value, a) == 0) ||
           (b != nullptr && std::strcmp(value, b) == 0) ||
           (c != nullptr && std::strcmp(value, c) == 0);
}

bool statusEquals(const char* status, const char* expected) {
    return status != nullptr && expected != nullptr && std::strcmp(status, expected) == 0;
}

}

namespace DisplayLogic {

DisplaySeverity severityForMetric(const char* metricName,
                                  float value,
                                  bool fresh,
                                  const char* status) {
    // Staleness takes priority over all other checks. A reading that has not
    // been refreshed within DisplayConfig::ValueTimeoutMs cannot be trusted
    // regardless of its numeric value, so show Timeout instead of a stale Ok.
    if (!fresh || statusEquals(status, "TIMEOUT")) {
        return DisplaySeverity::Timeout;
    }

    // Transport-level faults mean the value shown may be the last good reading,
    // not a current measurement. Treat these as Critical to alert the driver.
    if (statusEquals(status, "ERROR") || statusEquals(status, "SEND_FAIL")) {
        return DisplaySeverity::Critical;
    }

    // WARN and UNSUPPORTED indicate the ECU responded but the PID is unreliable
    // or optional on this vehicle. Surface a visual hint without raising a critical alert.
    if (statusEquals(status, "WARN") || statusEquals(status, "UNSUPPORTED")) {
        return DisplaySeverity::Warning;
    }

    if (metricName == nullptr) {
        return DisplaySeverity::Unknown;
    }

    if (equalsAny(metricName, "CoolantTemp", "EngineCoolantTemp", "05")) {
        if (value >= DisplayConfig::CoolantCriticalC) return DisplaySeverity::Critical;
        if (value >= DisplayConfig::CoolantWarnC) return DisplaySeverity::Warning;
        return DisplaySeverity::Ok;
    }

    if (equalsAny(metricName, "OilTemp", "EngineOilTemp", "5C")) {
        if (value >= DisplayConfig::OilCriticalC) return DisplaySeverity::Critical;
        if (value >= DisplayConfig::OilWarnC) return DisplaySeverity::Warning;
        return DisplaySeverity::Ok;
    }

    if (equalsAny(metricName, "BatteryVoltage", "ControlVoltage", "VOLTAGE")) {
        if (value <= DisplayConfig::VoltageCriticalLow ||
            value >= DisplayConfig::VoltageCriticalHigh) {
            return DisplaySeverity::Critical;
        }
        if (value <= DisplayConfig::VoltageWarnLow ||
            value > DisplayConfig::VoltageWarnHigh) {
            return DisplaySeverity::Warning;
        }
        return DisplaySeverity::Ok;
    }

    if (equalsAny(metricName, "RPM", "EngineRPM", "0C")) {
        if (value >= DisplayConfig::RpmCritical) return DisplaySeverity::Critical;
        if (value >= DisplayConfig::RpmWarn) return DisplaySeverity::Warning;
        return DisplaySeverity::Ok;
    }

    if (equalsAny(metricName, "BoostPressureBar", "Boost", "BOOST")) {
        if (value > DisplayConfig::BoostCriticalBar) return DisplaySeverity::Critical;
        if (value >= DisplayConfig::BoostWarnBar) return DisplaySeverity::Warning;
        return DisplaySeverity::Ok;
    }

    // The following metrics have no meaningful warn/critical thresholds (speed, load,
    // intake temp, fuel data, etc.). They are purely informational, so always
    // return Ok when fresh and not carrying a status-level override.
    if (equalsAny(metricName, "Speed", "VehicleSpeed", "0D") ||
        equalsAny(metricName, "EngineLoad", "Load", "04") ||
        equalsAny(metricName, "IntakeTemp", "IntakeAirTemp", "0F") ||
        equalsAny(metricName, "AverageConsumption", "AVG") ||
        equalsAny(metricName, "FuelRate", "EngineFuelRate", "5E") ||
        equalsAny(metricName, "Throttle", "ThrottlePosition", "11") ||
        equalsAny(metricName, "MAF", "MassAirFlow", "10") ||
        equalsAny(metricName, "FuelLevel", "2F") ||
        equalsAny(metricName, "RunTime", "Runtime", "1F") ||
        equalsAny(metricName, "AmbientTemp", "46") ||
        equalsAny(metricName, "ManifoldAbsolutePressure", "MAP", "0B") ||
        equalsAny(metricName, "BarometricPressure", "Baro", "33") ||
        equalsAny(metricName, "CANCount") ||
        equalsAny(metricName, "CAN", "OBD", "DTC")) {
        return DisplaySeverity::Ok;
    }

    return DisplaySeverity::Unknown;
}

const char* severityName(DisplaySeverity severity) {
    switch (severity) {
        case DisplaySeverity::Ok: return "Ok";
        case DisplaySeverity::Warning: return "Warning";
        case DisplaySeverity::Critical: return "Critical";
        case DisplaySeverity::Timeout: return "Timeout";
        case DisplaySeverity::Unknown: return "Unknown";
    }
    return "Unknown";
}

}
