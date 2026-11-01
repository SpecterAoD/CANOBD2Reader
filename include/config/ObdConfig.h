#pragma once

#include <cstddef>
#include <cstdint>

#include "PIDs.h"
#include "config/SenderConfig.h"

namespace ObdConfig {

inline constexpr uint8_t RequestedPids[] = {
    ENGINE_RPM,
    ENGINE_LOAD,
    ENGINE_COOLANT_TEMP,
    INTAKE_MANIFOLD_ABS_PRESSURE,
    VEHICLE_SPEED,
    INTAKE_AIR_TEMP,
    ABS_BAROMETRIC_PRESSURE,
    MAF_FLOW_RATE,
    THROTTLE_POSITION,
    FUEL_TANK_LEVEL_INPUT,
    RUN_TIME_SINCE_ENGINE_START,
    CONTROL_MODULE_VOLTAGE,
    AMBIENT_AIR_TEMP,
    ENGINE_OIL_TEMP
};

inline constexpr std::size_t ObdPidCount = sizeof(RequestedPids) / sizeof(RequestedPids[0]);

// Fast live values are the driver-facing values that must feel "live" on the
// display. They are queried in their own short round-robin so slow optional PIDs
// such as MAF, tank level or fuel rate cannot delay speed/RPM updates.
inline constexpr uint8_t FastLivePids[] = {
    ENGINE_RPM,
    VEHICLE_SPEED,
    ENGINE_LOAD,
    MAF_FLOW_RATE   // fuel consumption is derived from it
};
inline constexpr std::size_t FastLivePidCount = sizeof(FastLivePids) / sizeof(FastLivePids[0]);

// Slow live values still update regularly, but they are less important while
// driving and are often unsupported on some vehicles.
inline constexpr uint8_t SlowLivePids[] = {
    ENGINE_COOLANT_TEMP,
    CONTROL_MODULE_VOLTAGE,
    INTAKE_MANIFOLD_ABS_PRESSURE,
    INTAKE_AIR_TEMP,
    ABS_BAROMETRIC_PRESSURE,
    THROTTLE_POSITION,
    FUEL_TANK_LEVEL_INPUT,
    RUN_TIME_SINCE_ENGINE_START,
    AMBIENT_AIR_TEMP,
    ENGINE_OIL_TEMP
};
inline constexpr std::size_t SlowLivePidCount = sizeof(SlowLivePids) / sizeof(SlowLivePids[0]);

// PID 0x5E (engine fuel rate) is unreliable on VW TSI engines (first vehicle
// test: 2.2 L/h reported vs. ~7.7 L/h from the air mass). For petrol engines
// the rate is derived from MAF: fuel = air / AFR, zero in overrun (load ~0).
constexpr bool FuelRateFromMaf = true;
constexpr float StoichiometricAfr = 14.7f;       // petrol
constexpr float FuelDensityGramPerLitre = 745.0f; // petrol E5/E10
constexpr float OverrunLoadPercent = 2.0f;

constexpr float DefaultBarometricPressureKpa = SenderConfig::DefaultBarometricPressureKpa;
constexpr uint32_t DtcQueryIntervalMs = SenderConfig::DtcQueryIntervalMs;
constexpr uint32_t VinQueryIntervalMs = SenderConfig::VinQueryIntervalMs;

}
