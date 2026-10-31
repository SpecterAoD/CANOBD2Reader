#include "OBDHandler.h"
#include "LiveData.h"
#include "SenderLiveData.h"
#include "PIDs.h"
#include "PID_Converter.h"
#include "IsoTpHandler.h"
#include "BoostCalculator.h"
#include "ObdDiagnostics.h"
#include "SenderRuntimeState.h"
#include "Logger.h"

namespace {
IsoTp::IsoTpHandler isoTp;

float lastMapKpa = 0.0f;
float lastBarometricKpa = Obd::DefaultBarometricPressureKpa;
bool hasMapKpa = false;
bool hasBarometricKpa = false;

// Some vehicles ignore functional (broadcast) OBD requests on 0x7DF and only
// respond to physical (point-to-point) requests addressed directly to the ECU
// at 0x7E0. After a run of consecutive timeouts exceeds the configured threshold,
// we switch permanently for the session to avoid repeatedly broadcasting on a
// bus where only the unicast address reaches the ECU.
void updatePhysicalFallbackAfterFailure() {
    if (!SenderConfig::EnablePhysicalObdFallback) return;
    if (!Obd::Diagnostics::shouldSwitchToPhysicalFallback(SenderConfig::FunctionalTimeoutsBeforePhysicalFallback)) return;

    isoTp.setRequestId(IsoTp::PhysicalRequestId);
    Obd::Diagnostics::setPhysicalFallbackActive(true);
    Logger::warn("[OBD] Functional requests timed out repeatedly; switching to physical request id=0x7E0");
}

void updateBoostPressureIfReady() {
    if (!hasMapKpa) return;
    SenderLiveData::update(LiveData::Metric::BoostPressure,
                           Obd::calculateBoostPressureBarWithFallback(lastMapKpa, hasBarometricKpa, lastBarometricKpa));
}
}

bool OBD2Handler::sendRequest(uint8_t mode, uint8_t pid) {
    Obd::Diagnostics::recordRequest(mode, pid, isoTp.requestId());
    const bool ok = isoTp.sendRequest(mode, pid);
    if (!ok) {
        Obd::Diagnostics::recordSendFailure();
    }
    return ok;
}

bool OBD2Handler::sendRequestPayload(uint8_t mode, const uint8_t* payload, size_t payloadLength) {
    Obd::Diagnostics::recordRequest(mode, payload, payloadLength, isoTp.requestId());
    const bool ok = isoTp.sendRequest(mode, payload, payloadLength);
    if (!ok) {
        Obd::Diagnostics::recordSendFailure();
    }
    return ok;
}

bool OBD2Handler::receiveResponse(uint8_t mode, uint8_t pid, uint8_t* outData, uint8_t& outLen) {
    IsoTp::Payload response{};
    if (!receivePayload(mode, pid, response)) return false;
    if (response.length < 2) {
        Obd::Diagnostics::recordTimeout();
        updatePhysicalFallbackAfterFailure();
        return false;
    }

    const size_t payloadOffset = 2; // positive response mode + PID
    const size_t available = response.length - payloadOffset;
    outLen = static_cast<uint8_t>(available > 8 ? 8 : available);
    memcpy(outData, response.bytes.data() + payloadOffset, outLen);
    return true;
}

bool OBD2Handler::receivePayload(uint8_t mode, uint8_t pid, IsoTp::Payload& outPayload) {
    if (isoTp.receiveResponse(mode, pid, outPayload, SenderConfig::ObdResponseTimeoutMs)) {
        Obd::Diagnostics::recordValidResponse(outPayload.responseId,
                                              outPayload.bytes.data(),
                                              outPayload.length);
        return true;
    }

    if (isoTp.lastResponseWasNegative()) {
        Obd::Diagnostics::recordNegativeResponse(isoTp.lastNegativeService(), isoTp.lastNegativeCode());
    } else {
        Obd::Diagnostics::recordTimeout();
        updatePhysicalFallbackAfterFailure();
    }
    return false;
}

bool OBD2Handler::requestPayload(uint8_t mode,
                                 const uint8_t* payload,
                                 size_t payloadLength,
                                 uint8_t expectedPid,
                                 IsoTp::Payload& outPayload) {
    if (!sendRequestPayload(mode, payload, payloadLength)) return false;
    return receivePayload(mode, expectedPid, outPayload);
}

void OBD2Handler::calcConsumption(uint8_t pid, float value) {
    if (pid == VEHICLE_SPEED) Runtime::SenderRuntimeState::updateSpeed(value);
    if (pid == ENGINE_FUEL_RATE) Runtime::SenderRuntimeState::updateFuelRate(value);

    // Instantaneous consumption only makes sense while moving.
    if (SenderLiveData::has(LiveData::Metric::FuelRate) && SenderLiveData::has(LiveData::Metric::Speed)) {
        const float speed = SenderLiveData::value(LiveData::Metric::Speed);
        if (speed >= 5.0f) {
            SenderLiveData::update(LiveData::Metric::InstantConsumption,
                                   SenderLiveData::value(LiveData::Metric::FuelRate) * 100.0f / speed);
        }
    }

    if (!Runtime::SenderRuntimeState::hasConsumptionInput()) return;
    float avg = 0.0f;
    if (Runtime::SenderRuntimeState::addConsumptionSample(avg)) {
        SenderLiveData::update(LiveData::Metric::AverageConsumption, avg);
    }
}

namespace {
void updateActivityMetrics(uint8_t pid, float value) {
    if (pid == ENGINE_RPM) Runtime::SenderRuntimeState::updateRpm(value);
    if (pid == VEHICLE_SPEED) Runtime::SenderRuntimeState::updateSpeed(value);
    if (pid == ENGINE_LOAD) Runtime::SenderRuntimeState::updateEngineLoad(value);
    if (pid == THROTTLE_POSITION) Runtime::SenderRuntimeState::updateThrottle(value);
    if (pid == ENGINE_FUEL_RATE) Runtime::SenderRuntimeState::updateFuelRate(value);
}
}

void OBD2Handler::updateBoostPressure(uint8_t pid, float value) {
    if (pid == INTAKE_MANIFOLD_ABS_PRESSURE) {
        lastMapKpa = value;
        hasMapKpa = true;
        updateBoostPressureIfReady();
        return;
    }

    if (pid == ABS_BAROMETRIC_PRESSURE) {
        lastBarometricKpa = value;
        hasBarometricKpa = true;
        updateBoostPressureIfReady();
    }
}

bool OBD2Handler::requestAndSendPID(uint8_t pid) {
    if (!sendRequest(read_LiveData, pid)) {
        Logger::warn((String("[OBD] Senden fehlgeschlagen pid=0x") + String(pid, HEX)).c_str());
        return false;
    }

    uint8_t responseData[8];
    uint8_t responseLen = 0;
    if (!receiveResponse(read_LiveData, pid, responseData, responseLen)) {
        // No text packet: the display sees a stale value via the snapshot age,
        // details are in the OBD diagnostics counters and the log.
        const String message = lastResponseWasNegative()
            ? String("[OBD] Negative Antwort pid=0x") + String(pid, HEX) + " nrc=0x" + String(lastNegativeCode(), HEX)
            : String("[OBD] Timeout pid=0x") + String(pid, HEX);
        Logger::warn(message.c_str());
        return false;
    }

    Logger::obdFrame(pid, responseData, responseLen);
    const PIDResult result = convertPID(pid, responseData, responseLen);
    updateActivityMetrics(pid, result.value);

    LiveData::Metric metric;
    if (LiveData::metricForPid(pid, metric)) SenderLiveData::update(metric, result.value);

    if (pid == VEHICLE_SPEED || pid == ENGINE_FUEL_RATE) calcConsumption(pid, result.value);
    if (pid == INTAKE_MANIFOLD_ABS_PRESSURE || pid == ABS_BAROMETRIC_PRESSURE) updateBoostPressure(pid, result.value);

    Logger::obdValue(getPIDName(pid), result.value, result.unit);
    return true;
}

bool OBD2Handler::lastResponseWasNegative() {
    return isoTp.lastResponseWasNegative();
}

uint8_t OBD2Handler::lastNegativeService() {
    return isoTp.lastNegativeService();
}

uint8_t OBD2Handler::lastNegativeCode() {
    return isoTp.lastNegativeCode();
}

uint32_t OBD2Handler::activeRequestId() {
    return isoTp.requestId();
}
