#pragma once

#include <cstddef>
#include <cstdint>
#include "config/LoggingConfig.h"

namespace SenderConfig {
constexpr bool EnableOBD2 = true;
constexpr bool EnableUDS = true;
constexpr bool SendRawData = true;
constexpr bool RequireWebStart = false;

constexpr int LedPin1 = 26;
constexpr int LedPin2 = 27;
constexpr int ButtonPin = 25;
constexpr int CanRxPin = 4;
constexpr int CanTxPin = 5;
constexpr int VoltageDividerPin = 32;

constexpr int PollingRateMs = 10;
constexpr int CanIdleTimeoutMs = 500;
// Target refresh per PID (see Obd::PidPollPlanner). The planner always asks
// for the most overdue PID, so slow values cannot be starved.
constexpr uint32_t FastPidIntervalMs = 200;
constexpr uint32_t SlowPidIntervalMs = 2000;
// Complete live table to the display, independent of the polling rhythm.
constexpr uint32_t LiveSnapshotIntervalMs = 100;
constexpr uint32_t ObdResponseTimeoutMs = 120;
constexpr uint32_t ObdTxTimeoutMs = 20;
constexpr uint32_t BatterySendIntervalMs = 3000;
constexpr uint32_t PowerTelemetryIntervalMs = 1000;
constexpr uint32_t HeartbeatIntervalMs = 1000;
constexpr uint32_t SimulationIntervalMs = 250;
constexpr std::size_t SimulationStatusFramesPerTick = 2;
constexpr uint32_t RawCanTelemetryIntervalMs = 250;
constexpr uint32_t LedTestDebounceMs = 50;
constexpr uint32_t SupportedPidRefreshMs = 60000;
constexpr uint32_t SupportedPidRetryMs = 2000;
// Consecutive OBD timeouts after which live polling pauses (ECU asleep).
constexpr uint32_t TimeoutsBeforeObdIdle = 10;
constexpr uint32_t DtcQueryIntervalMs = 30000;
constexpr uint32_t VinQueryIntervalMs = 120000;
constexpr uint32_t UdsQueryIntervalMs = 60000;
constexpr uint32_t UdsResponseTimeoutMs = 350;
constexpr uint32_t UdsResponsePendingTimeoutMs = 4000;
constexpr bool EnablePhysicalObdFallback = true;
constexpr uint8_t FunctionalTimeoutsBeforePhysicalFallback = 3;
constexpr int CpuFrequency = 160;

// Persistent sender diagnostic log. This is intentionally runtime-only data
// stored in SPIFFS so OBD/CAN/ESP-NOW problems can be inspected after a drive.
constexpr bool EnablePersistentDiagnosticLog = LoggingConfig::EnablePersistentDiagnosticLog;
constexpr bool PersistTelemetryPayloadsToDiagnosticLog = LoggingConfig::PersistTelemetryPayloadsToDiagnosticLog;
constexpr std::size_t DiagnosticLogMaxBytes = LoggingConfig::DiagnosticLogMaxBytes;
constexpr const char* DiagnosticLogPath = LoggingConfig::SenderDiagnosticLogPath;
constexpr const char* DiagnosticLogArchivePath = LoggingConfig::SenderDiagnosticLogArchivePath;
constexpr uint32_t TwaiStatusLogIntervalMs = LoggingConfig::TwaiStatusLogIntervalMs;
constexpr std::size_t MaxCanFramesPerTick = 16;

constexpr float VoltageCalcFactor = 4.81f;
constexpr float DefaultBarometricPressureKpa = 101.3f;

constexpr std::size_t CanRxQueueLength = 32;
constexpr std::size_t CanTxQueueLength = 4;
constexpr uint8_t IsoTpBlockSize = 0;
constexpr uint8_t IsoTpStMinMs = 0;
}
