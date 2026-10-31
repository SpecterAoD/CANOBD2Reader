#pragma once

#include <cstdint>

namespace PowerConfig {

constexpr bool EnablePowerManager = true;
constexpr bool EnableStartStopDetection = true;
// Tuned for VW MQB Evo (Tiguan III): ECUs and bus traffic stay alive for
// minutes after ignition-off and start-stop is used aggressively.
constexpr uint32_t ParkDetectionTimeoutMs = 5UL * 60UL * 1000UL;
constexpr uint32_t DisplaySleepAfterMs = 10UL * 60UL * 1000UL;
constexpr uint32_t WakeupDebounceMs = 2000;
// A button press counts as user activity for this long.
constexpr uint32_t UserActivityWindowMs = 3000;

constexpr float EngineRunningRpm = 400.0f;
constexpr float MovingSpeedKph = 1.0f;
constexpr float ActiveEngineLoadPercent = 5.0f;
constexpr float ActiveThrottlePercent = 2.0f;
constexpr float BoardVoltageAwakeMin = 12.0f;

} // namespace PowerConfig
