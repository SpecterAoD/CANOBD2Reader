#pragma once

#include <Arduino.h>
#include <cstdint>

namespace Runtime {

/// Runtime snapshot shown by the sender web console.
///
/// This deliberately contains only volatile status. Static configuration stays
/// in include/config/*, while the web layer receives a fresh copy of this state
/// from SenderApp on every loop iteration.
struct WebRuntimeStatus {
    bool canActive = false;
    bool obdActive = false;
    bool pidSupportReady = false;
    bool simulationActive = false;
    String simulationScenario = "NormalSingleFrame";
    float batteryVoltage = 0.0f;
    uint32_t uptimeMs = 0;
    uint32_t telemetrySequence = 0;
    uint32_t telemetrySendOk = 0;
    uint32_t telemetrySendFail = 0;
    uint32_t heartbeatCount = 0;
    uint32_t lastCanAgeMs = 0;
    uint32_t lastObdAgeMs = 0;
    String canState = "UNKNOWN";
    String obdState = "UNKNOWN";
    String espNowState = "UNKNOWN";
    String ledState = "Booting";
    bool ledTestActive = false;
    bool vehicleOff = false;
    uint32_t ledLastStateChangeAt = 0;
    String vehicleState = "Booting";
    String powerCommand = "None";
    uint8_t activityScore = 0;
    bool startStopDetected = false;
    bool parkedDetected = false;
    uint32_t parkedStartedAtMs = 0;
    uint32_t displaySleepDueAtMs = 0;
    uint32_t lastWakeupAtMs = 0;
    uint32_t lastSleepAtMs = 0;
    String lastSendError = "";
    String lastDtc = "--";
    String lastVin = "--";
    String lastTelemetry = "--";
    String lastError = "";
    uint32_t obdRequestCount = 0;
    uint32_t obdSendFailureCount = 0;
    uint32_t obdTimeoutCount = 0;
    uint32_t obdValidResponseCount = 0;
    uint32_t obdNegativeResponseCount = 0;
    uint32_t obdTimeoutStreak = 0;
    bool obdPhysicalFallbackActive = false;
    uint32_t obdRequestCanId = 0x7DF;
    uint32_t supportedPidMask01_20 = 0;
    uint32_t supportedPidMask21_40 = 0;
    uint32_t supportedPidMask41_60 = 0;
    String lastObdRequest = "--";
    String lastEcuResponse = "--";
    String lastNegativeResponse = "--";
    bool udsAvailable = false;
    uint32_t udsRequestCount = 0;
    uint32_t udsSendFailureCount = 0;
    uint32_t udsTimeoutCount = 0;
    uint32_t udsPositiveResponseCount = 0;
    uint32_t udsNegativeResponseCount = 0;
    String lastUdsRequest = "--";
    String lastUdsResponse = "--";
    String lastUdsNegativeResponse = "--";
    String lastUdsDid = "--";
    String lastUdsDtc = "--";
};

} // namespace Runtime
