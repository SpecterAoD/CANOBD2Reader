#pragma once

#include <cstddef>
#include <cstdint>

namespace SenderConfig {
constexpr bool EnableCAN = true;
constexpr bool EnableOBD2 = true;
constexpr bool SendRawData = false;
constexpr bool SendRawDataOnly = false;
constexpr bool RequireWebStart = false;

constexpr int LedPin1 = 26;
constexpr int LedPin2 = 27;
constexpr int ButtonPin = 25;
constexpr int CanRxPin = 4;
constexpr int CanTxPin = 5;
constexpr int VoltageDividerPin = 32;

constexpr uint32_t CanBitrate = 500000;
constexpr int PollingRateMs = 500;
constexpr int CanIdleTimeoutMs = 500;
constexpr int SleepPeriodSec = 6;
constexpr uint32_t ObdPollIntervalMs = 200;
constexpr uint32_t ObdResponseTimeoutMs = 250;
constexpr uint32_t ObdTxTimeoutMs = 50;
constexpr uint32_t BatterySendIntervalMs = 3000;
constexpr uint32_t HeartbeatIntervalMs = 1000;
constexpr uint32_t SimulationIntervalMs = 250;
constexpr uint32_t LedTestDebounceMs = 50;
constexpr uint32_t SupportedPidRefreshMs = 60000;
constexpr uint32_t DtcQueryIntervalMs = 30000;
constexpr uint32_t StartStopDelayMs = 300000;
constexpr int CpuFrequency = 80;

constexpr float VoltageCalcFactor = 4.81f;
constexpr float VoltageChangeThreshold = 0.2f;
constexpr float DefaultBarometricPressureKpa = 101.3f;

constexpr std::size_t CanRxQueueLength = 32;
constexpr std::size_t CanTxQueueLength = 4;
constexpr uint8_t IsoTpBlockSize = 0;
constexpr uint8_t IsoTpStMinMs = 0;
constexpr uint32_t IsoTpConsecutiveFrameTimeoutMs = 300;
}
