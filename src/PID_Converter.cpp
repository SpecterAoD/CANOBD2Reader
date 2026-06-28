#include "PID_Converter.h"
#include "PIDs.h"

// ==== Einzelne Konvertierer (je nach PID) ====

namespace {
    PIDResult convertENGINE_LOAD(const byte* d) {
        return {(d[0] * 100.0f) / 255.0f, "%"};
    }

    PIDResult convertENGINE_COOLANT_TEMP(const byte* d) {
        return {d[0] - 40.0f, "°C"};
    }

    PIDResult convertFUEL_PRESSURE(const byte* d) {
        return {d[0] * 3.0f, "kPa"};
    }

    PIDResult convertINTAKE_MANIFOLD_ABS_PRESSURE(const byte* d) {
        return {static_cast<float>(d[0]), "kPa"};
    }

    PIDResult convertENGINE_RPM(const byte* d) {
        return {((d[0] * 256.0f) + d[1]) / 4.0f, "rpm"};
    }

    PIDResult convertVEHICLE_SPEED(const byte* d) {
        return {static_cast<float>(d[0]), "km/h"};
    }

    PIDResult convertTIMING_ADVANCE(const byte* d) {
        return {(d[0] / 2.0f) - 64.0f, "° BTDC"};
    }

    PIDResult convertINTAKE_AIR_TEMP(const byte* d) {
        return {d[0] - 40.0f, "°C"};
    }

    PIDResult convertMAF_FLOW_RATE(const byte* d) {
        return {((d[0] * 256.0f) + d[1]) / 100.0f, "g/s"};
    }

    PIDResult convertTHROTTLE_POSITION(const byte* d) {
        return {(d[0] * 100.0f) / 255.0f, "%"};
    }

    PIDResult convertFUEL_TANK_LEVEL_INPUT(const byte* d) {
        return {(d[0] * 100.0f) / 255.0f, "%"};
    }

    PIDResult convertCONTROL_MODULE_VOLTAGE(const byte* d) {
        return {((d[0] * 256.0f) + d[1]) / 1000.0f, "V"};
    }

    PIDResult convertENGINE_OIL_TEMP(const byte* d) {
        return {d[0] - 40.0f, "°C"};
    }

    PIDResult convertENGINE_FUEL_RATE(const byte* d) {
        return {((d[0] * 256.0f) + d[1]) / 20.0f, "L/h"};
    }

    PIDResult convertRUN_TIME_SINCE_ENGINE_START(const byte* d) {
        return {((d[0] * 256.0f) + d[1]), "s"};
    }

    PIDResult convertAMBIENT_AIR_TEMP(const byte* d) {
        return {d[0] - 40.0f, "°C"};
    }

    PIDResult unknownPID() {
        return {-1.0f, "N/A"};
    }

    PIDResult invalidLength() {
        return {-1.0f, "INVALID"};
    }

    typedef PIDResult (*ConverterFunc)(const byte*);
    struct PIDConverterEntry {
        byte pid;
        ConverterFunc func;
        byte minLength;
    };

    const PIDConverterEntry converterTable[] = {
        {ENGINE_LOAD, convertENGINE_LOAD, 1},
        {ENGINE_COOLANT_TEMP, convertENGINE_COOLANT_TEMP, 1},
        {FUEL_PRESSURE, convertFUEL_PRESSURE, 1},
        {INTAKE_MANIFOLD_ABS_PRESSURE, convertINTAKE_MANIFOLD_ABS_PRESSURE, 1},
        {ENGINE_RPM, convertENGINE_RPM, 2},
        {VEHICLE_SPEED, convertVEHICLE_SPEED, 1},
        {TIMING_ADVANCE, convertTIMING_ADVANCE, 1},
        {INTAKE_AIR_TEMP, convertINTAKE_AIR_TEMP, 1},
        {MAF_FLOW_RATE, convertMAF_FLOW_RATE, 2},
        {THROTTLE_POSITION, convertTHROTTLE_POSITION, 1},
        {RUN_TIME_SINCE_ENGINE_START, convertRUN_TIME_SINCE_ENGINE_START, 2},
        {FUEL_TANK_LEVEL_INPUT, convertFUEL_TANK_LEVEL_INPUT, 1},
        {CONTROL_MODULE_VOLTAGE, convertCONTROL_MODULE_VOLTAGE, 2},
        {AMBIENT_AIR_TEMP, convertAMBIENT_AIR_TEMP, 1},
        {ENGINE_OIL_TEMP, convertENGINE_OIL_TEMP, 1},
        {ENGINE_FUEL_RATE, convertENGINE_FUEL_RATE, 2},
    };

    const size_t converterTableSize = sizeof(converterTable) / sizeof(PIDConverterEntry);
}

// ==== Hauptumrechnungsfunktion ====

PIDResult convertPID(byte pid, const byte* data, byte length) {
    for (size_t i = 0; i < converterTableSize; ++i) {
        if (converterTable[i].pid == pid) {
            if (length < converterTable[i].minLength) return invalidLength();
            return converterTable[i].func(data);
        }
    }
    return unknownPID();
}

// ==== PID-Name Mapping ====

const char* getPIDName(byte pid) {
    switch (pid) {
        case SUPPORTED_PIDS_1_20:              return "SupportedPIDs";
        case MONITOR_STATUS_SINCE_DTC_CLEARED: return "MonitorStatus";
        case FREEZE_DTC:                       return "FreezeDTC";
        case FUEL_SYSTEM_STATUS:               return "FuelSystemStatus";
        case ENGINE_LOAD:                      return "EngineLoad";
        case ENGINE_COOLANT_TEMP:              return "CoolantTemp";
        case SHORT_TERM_FUEL_TRIM_BANK_1:      return "ShortFT_B1";
        case LONG_TERM_FUEL_TRIM_BANK_1:       return "LongFT_B1";
        case SHORT_TERM_FUEL_TRIM_BANK_2:      return "ShortFT_B2";
        case LONG_TERM_FUEL_TRIM_BANK_2:       return "LongFT_B2";
        case FUEL_PRESSURE:                    return "FuelPressure";
        case INTAKE_MANIFOLD_ABS_PRESSURE:     return "IntakePressure";
        case ENGINE_RPM:                       return "RPM";
        case VEHICLE_SPEED:                    return "Speed";
        case TIMING_ADVANCE:                   return "TimingAdvance";
        case INTAKE_AIR_TEMP:                  return "IntakeTemp";
        case MAF_FLOW_RATE:                    return "MAF";
        case THROTTLE_POSITION:                return "Throttle";
        case RUN_TIME_SINCE_ENGINE_START:      return "RunTime";
        case FUEL_TANK_LEVEL_INPUT:            return "FuelLevel";
        case CONTROL_MODULE_VOLTAGE:           return "ControlVoltage";
        case AMBIENT_AIR_TEMP:                 return "AmbientTemp";
        case ENGINE_OIL_TEMP:                  return "OilTemp";
        case ENGINE_FUEL_RATE:                 return "FuelRate";
        default:                               return "Unknown";
    }
}
