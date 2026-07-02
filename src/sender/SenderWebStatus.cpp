#include "SenderWebStatus.h"

#include <Arduino.h>

#include "ObdDiagnostics.h"
#include "SenderObdScheduler.h"
#include "SenderPower.h"
#include "SenderTelemetry.h"
#include "SenderUdsScheduler.h"
#include "RuntimeSimulation.h"
#include "UdsDiagnostics.h"
#include "config/DisplayConfig.h"
#include "config/SenderConfig.h"

namespace SenderWebStatus {

Runtime::WebRuntimeStatus build(const Input& input) {
    Runtime::WebRuntimeStatus status;
    const uint32_t now = millis();
    const uint32_t lastObdResponseAt = SenderTelemetry::lastObdResponseAt();

    status.canActive = input.canBusActive;
    status.obdActive = SenderConfig::EnableOBD2 &&
                       lastObdResponseAt > 0 &&
                       now - lastObdResponseAt <= DisplayConfig::ObdTimeoutMs;
    status.pidSupportReady = SenderObdScheduler::supportedPidsInitialized();
    status.simulationActive = Simulation::RuntimeSimulation::enabled();
    status.simulationScenario = Simulation::RuntimeSimulation::scenarioName();
    status.batteryVoltage = SenderPower::getLastVoltage();
    status.uptimeMs = now;
    status.telemetrySequence = SenderTelemetry::sequence();
    status.telemetrySendOk = SenderTelemetry::sendOkCount();
    status.telemetrySendFail = SenderTelemetry::sendFailCount();
    status.heartbeatCount = input.heartbeatCount;
    status.lastCanAgeMs = input.lastCanMessageAt == 0 ? 0 : now - input.lastCanMessageAt;
    status.lastObdAgeMs = lastObdResponseAt == 0 ? 0 : now - lastObdResponseAt;
    status.canState = input.canDriverReady ? (input.canBusActive ? "ACTIVE" : "IDLE") : "INIT_FAIL";
    status.obdState = SenderConfig::EnableOBD2 ? (status.obdActive ? "ACTIVE" : "NO_RESPONSE") : "DISABLED";
    status.espNowState = input.espNowReady ? "READY" : "INIT_FAIL";
    status.lastSendError = SenderTelemetry::lastSendError();
    status.lastDtc = SenderObdScheduler::lastDtcText();
    status.lastVin = SenderObdScheduler::lastVinText();
    status.lastTelemetry = String(SenderTelemetry::lastPayload());
    status.lastError = SenderTelemetry::lastError();

    status.obdRequestCount = Obd::Diagnostics::requestCount();
    status.obdSendFailureCount = Obd::Diagnostics::sendFailureCount();
    status.obdTimeoutCount = Obd::Diagnostics::timeoutCount();
    status.obdValidResponseCount = Obd::Diagnostics::validResponseCount();
    status.obdNegativeResponseCount = Obd::Diagnostics::negativeResponseCount();
    status.obdTimeoutStreak = Obd::Diagnostics::timeoutStreak();
    status.obdPhysicalFallbackActive = Obd::Diagnostics::physicalFallbackActive();
    status.obdRequestCanId = Obd::Diagnostics::requestCanId();
    status.supportedPidMask01_20 = Obd::Diagnostics::supportedPidMask(0);
    status.supportedPidMask21_40 = Obd::Diagnostics::supportedPidMask(1);
    status.supportedPidMask41_60 = Obd::Diagnostics::supportedPidMask(2);
    status.lastObdRequest = Obd::Diagnostics::lastRequest();
    status.lastEcuResponse = Obd::Diagnostics::lastEcuResponse();
    status.lastNegativeResponse = Obd::Diagnostics::lastNegativeResponse();

    status.udsAvailable = Uds::Diagnostics::available();
    status.udsRequestCount = Uds::Diagnostics::requestCount();
    status.udsSendFailureCount = Uds::Diagnostics::sendFailureCount();
    status.udsTimeoutCount = Uds::Diagnostics::timeoutCount();
    status.udsPositiveResponseCount = Uds::Diagnostics::positiveResponseCount();
    status.udsNegativeResponseCount = Uds::Diagnostics::negativeResponseCount();
    status.lastUdsRequest = Uds::Diagnostics::lastRequest();
    status.lastUdsResponse = Uds::Diagnostics::lastResponse();
    status.lastUdsNegativeResponse = Uds::Diagnostics::lastNegativeResponse();
    status.lastUdsDid = SenderUdsScheduler::lastDidText();
    status.lastUdsDtc = SenderUdsScheduler::lastDtcText();

    return status;
}

} // namespace SenderWebStatus
