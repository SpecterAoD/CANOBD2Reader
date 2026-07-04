#pragma once

#include <cstdint>

namespace PowerConfig {

enum class VehiclePowerProfile : uint8_t {
    Generic,
    VolkswagenMQB,
    VolkswagenMQBEvo,
    BMW,
    Mercedes,
    Toyota,
    Custom
};

constexpr bool EnablePowerManager = true;
constexpr bool EnableStartStopDetection = true;
constexpr uint32_t ParkDetectionTimeoutMs = 5UL * 60UL * 1000UL;
constexpr uint32_t DisplaySleepAfterMs = 10UL * 60UL * 1000UL;
constexpr uint32_t WakeupDebounceMs = 2000;
constexpr bool DimDisplayWhileIdle = true;
constexpr uint8_t IdleBrightnessPercent = 25;
constexpr bool EnableActivityScore = true;

// The user's current vehicle is a VW Tiguan III / MQB Evo. These cars can keep
// ECUs and bus traffic alive for several minutes after ignition-off, and they
// use start-stop aggressively. The profile is kept configurable for later
// vehicle-specific tuning without changing the ActivityMonitor state machine.
constexpr VehiclePowerProfile ActiveProfile = VehiclePowerProfile::VolkswagenMQBEvo;

constexpr float EngineRunningRpm = 400.0f;
constexpr float MovingSpeedKph = 1.0f;
constexpr float ActiveEngineLoadPercent = 5.0f;
constexpr float ActiveThrottlePercent = 2.0f;
constexpr float BoardVoltageAwakeMin = 12.0f;

} // namespace PowerConfig
