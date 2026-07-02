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
    ENGINE_OIL_TEMP,
    ENGINE_FUEL_RATE
};

inline constexpr std::size_t ObdPidCount = sizeof(RequestedPids) / sizeof(RequestedPids[0]);
inline constexpr std::size_t PidCount = ObdPidCount;

constexpr float DefaultBarometricPressureKpa = SenderConfig::DefaultBarometricPressureKpa;
constexpr uint32_t SupportedPidRefreshIntervalMs = SenderConfig::SupportedPidRefreshMs;
constexpr uint32_t DtcQueryIntervalMs = SenderConfig::DtcQueryIntervalMs;
constexpr uint32_t VinQueryIntervalMs = SenderConfig::VinQueryIntervalMs;

}
