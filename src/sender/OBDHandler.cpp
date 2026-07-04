#include "OBDHandler.h"
#include "Utils.h"
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

void updatePhysicalFallbackAfterFailure() {
    if (!SenderConfig::EnablePhysicalObdFallback) return;
    if (!Obd::Diagnostics::shouldSwitchToPhysicalFallback(SenderConfig::FunctionalTimeoutsBeforePhysicalFallback)) return;

    isoTp.setRequestId(IsoTp::PhysicalRequestId);
    Obd::Diagnostics::setPhysicalFallbackActive(true);
    Logger::warn("[OBD] Functional requests timed out repeatedly; switching to physical request id=0x7E0");
}

void sendBoostPressureIfReady() {
    if (!hasMapKpa) return;

    const float boostBar = Obd::calculateBoostPressureBarWithFallback(lastMapKpa,
                                                                      hasBarometricKpa,
                                                                      lastBarometricKpa);
    char valueText[16];
    snprintf(valueText, sizeof(valueText), "%.2f", static_cast<double>(boostBar));
    Utils::sendTelemetry("OBD", "BOOST", "BoostPressureBar", valueText, "bar", "OK");
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

    if (!Runtime::SenderRuntimeState::hasConsumptionInput()) return;

    float avg = 0.0f;
    if (Runtime::SenderRuntimeState::addConsumptionSample(avg)) {
        char avgText[16];
        snprintf(avgText, sizeof(avgText), "%.2f", static_cast<double>(avg));
        Utils::sendTelemetry("FUEL", "AVG", "AverageConsumption", avgText, "L/100km", "OK");
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
        sendBoostPressureIfReady();
        return;
    }

    if (pid == ABS_BAROMETRIC_PRESSURE) {
        lastBarometricKpa = value;
        hasBarometricKpa = true;
        sendBoostPressureIfReady();
    }
}

bool OBD2Handler::requestAndSendPID(uint8_t pid) {
    if (!sendRequest(read_LiveData, pid)) {
        const String message = String("[OBD] Timeout waiting for ECU response pid=0x") +
                               String(pid, HEX) + " send failed";
        Logger::warn(message.c_str());
        Utils::sendOBDError(pid, getPIDName(pid));
        return false;
    }

    uint8_t responseData[8];
    uint8_t responseLen = 0;

    if (receiveResponse(read_LiveData, pid, responseData, responseLen)) {
        Logger::obdFrame(pid, responseData, responseLen);
        if constexpr (SenderConfig::SendRawDataOnly) {
            Utils::sendRawOBDData(pid, responseData, responseLen);
        } else {
            PIDResult result = convertPID(pid, responseData, responseLen);
            updateActivityMetrics(pid, result.value);

            if (pid == VEHICLE_SPEED || pid == ENGINE_FUEL_RATE) {
                calcConsumption(pid, result.value);
            }

            if (pid == INTAKE_MANIFOLD_ABS_PRESSURE || pid == ABS_BAROMETRIC_PRESSURE) {
                updateBoostPressure(pid, result.value);
            }

            Logger::obdValue(getPIDName(pid), result.value, result.unit);
            Utils::sendOBDValue(pid, getPIDName(pid), result.value, result.unit);
        }
        return true;
    } else {
        const String message = lastResponseWasNegative()
            ? String("[OBD] Negative response pid=0x") + String(pid, HEX) +
              " service=0x" + String(lastNegativeService(), HEX) +
              " nrc=0x" + String(lastNegativeCode(), HEX)
            : String("[OBD] Timeout waiting for ECU response pid=0x") + String(pid, HEX);
        Logger::warn(message.c_str());
        if (lastResponseWasNegative()) {
            char pidHex[4];
            char nrcText[24];
            snprintf(pidHex, sizeof(pidHex), "%02X", pid);
            snprintf(nrcText, sizeof(nrcText), "NRC 0x%02X", lastNegativeCode());
            Utils::sendTelemetry("OBD", pidHex, getPIDName(pid), nrcText, "", "ERROR");
        } else {
            Utils::sendOBDError(pid, getPIDName(pid));
        }
        return false;
    }
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
