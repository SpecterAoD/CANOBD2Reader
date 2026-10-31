#include "SenderObdScheduler.h"

#include "config/ObdConfig.h"
#include "config/SenderConfig.h"
#include "config/DisplayConfig.h"
#include "PIDs.h"
#include "PID_Converter.h"
#include "OBDHandler.h"
#include "ObdDiagnostics.h"
#include "DtcDecoder.h"
#include "VinDecoder.h"
#include "IsoTpTypes.h"
#include "WebConsoleHandler.h"
#include "LiveData.h"
#include "PidPollPlanner.h"
#include "SenderLiveData.h"

namespace {
uint32_t lastObdPollAt = 0;
uint32_t lastSupportedPidRefreshAt = 0;
uint32_t lastDtcQueryAt = 0;
uint32_t lastVinQueryAt = 0;
bool supportedInitialized = false;
Obd::PidPollPlanner planner;
uint32_t supportedPids01_20 = 0;
uint32_t supportedPids21_40 = 0;
uint32_t supportedPids41_60 = 0;
String dtcText = "--";
String vinText = "--";

uint32_t bytesToMask(const byte* data, byte length) {
    if (length < 4) return 0;
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           static_cast<uint32_t>(data[3]);
}

bool querySupportedPidRange(byte rangePid, uint32_t& targetMask, uint32_t nowMs,
                            uint32_t& lastCanMessageAt,
                            uint32_t& lastObdResponseAt,
                            bool& canBusActive) {
    byte responseData[8];
    byte responseLen = 0;

    if (!OBD2Handler::sendRequest(read_LiveData, rangePid)) return false;
    if (!OBD2Handler::receiveResponse(read_LiveData, rangePid, responseData, responseLen)) return false;

    lastCanMessageAt = nowMs;
    lastObdResponseAt = nowMs;
    canBusActive = true;
    targetMask = bytesToMask(responseData, responseLen);
    return targetMask != 0;
}

bool isPidSupported(byte pid) {
    if (!supportedInitialized) return true;

    byte rangeBase = 0x00;
    uint32_t mask = supportedPids01_20;
    if (pid >= 0x21 && pid <= 0x40) {
        rangeBase = 0x20;
        mask = supportedPids21_40;
    } else if (pid >= 0x41 && pid <= 0x60) {
        rangeBase = 0x40;
        mask = supportedPids41_60;
    } else if (pid < 0x01 || pid > 0x20) {
        return false;
    }

    const byte offset = pid - rangeBase;
    if (offset == 0 || offset > 32) return false;
    return (mask & (1UL << (32 - offset))) != 0;
}

void buildPlanner() {
    planner.clear();
    for (std::size_t index = 0; index < ObdConfig::FastLivePidCount; ++index) {
        planner.add(ObdConfig::FastLivePids[index], SenderConfig::FastPidIntervalMs);
    }
    for (std::size_t index = 0; index < ObdConfig::SlowLivePidCount; ++index) {
        planner.add(ObdConfig::SlowLivePids[index], SenderConfig::SlowPidIntervalMs);
    }
}

// Applies the ECU's supported-PID masks: unsupported PIDs are no longer polled
// and shown as "nicht unterstuetzt" on the display instead of "wartet".
void applySupportMasks() {
    auto apply = [](const uint8_t* pids, std::size_t count) {
        for (std::size_t index = 0; index < count; ++index) {
            const byte pid = pids[index];
            const bool supported = isPidSupported(pid);
            planner.setSupported(pid, supported);
            LiveData::Metric metric;
            if (!supported && LiveData::metricForPid(pid, metric)) SenderLiveData::markUnsupported(metric);
        }
    };
    apply(ObdConfig::FastLivePids, ObdConfig::FastLivePidCount);
    apply(ObdConfig::SlowLivePids, ObdConfig::SlowLivePidCount);
}

void refreshSupportedPids(uint32_t nowMs,
                          uint32_t& lastCanMessageAt,
                          uint32_t& lastObdResponseAt,
                          bool& canBusActive,
                          SenderCallbacks::SendStatus sendStatus) {
    bool ok = querySupportedPidRange(SUPPORTED_PIDS_1_20, supportedPids01_20,
                                     nowMs, lastCanMessageAt, lastObdResponseAt, canBusActive);

    if (ok && (supportedPids01_20 & 0x00000001UL)) {
        ok = querySupportedPidRange(SUPPORTED_PIDS_21_40, supportedPids21_40,
                                    nowMs, lastCanMessageAt, lastObdResponseAt, canBusActive);
    }

    if (ok && (supportedPids21_40 & 0x00000001UL)) {
        ok = querySupportedPidRange(SUPPORTED_PIDS_41_60, supportedPids41_60,
                                    nowMs, lastCanMessageAt, lastObdResponseAt, canBusActive);
    }

    supportedInitialized = ok;
    if (ok) applySupportMasks();
    Obd::Diagnostics::setSupportedPidMasks(supportedPids01_20,
                                           supportedPids21_40,
                                           supportedPids41_60,
                                           ok);
    if (ok) {
        char masks[112];
        snprintf(masks, sizeof(masks), "[OBD] Supported PIDs 01-20=0x%08lX 21-40=0x%08lX 41-60=0x%08lX",
                 static_cast<unsigned long>(supportedPids01_20),
                 static_cast<unsigned long>(supportedPids21_40),
                 static_cast<unsigned long>(supportedPids41_60));
        WebConsoleHandler::log(masks);
    } else {
        WebConsoleHandler::log("[OBD] Supported PID query failed");
    }

    if (sendStatus != nullptr) sendStatus("PID_SUPPORT", ok ? "READY" : "UNKNOWN", ok ? "OK" : "WARN");
}

void queryAndSendDtc(uint32_t nowMs,
                     uint32_t& lastCanMessageAt,
                     uint32_t& lastObdResponseAt,
                     bool& canBusActive,
                     SenderCallbacks::SendTelemetry sendTelemetry) {
    const uint8_t payload[] = {read_DTCs};
    IsoTp::Payload response{};

    if (!OBD2Handler::requestPayload(read_DTCs, payload, sizeof(payload), 0xFF, response)) {
        if (OBD2Handler::lastResponseWasNegative()) {
            dtcText = Obd::Diagnostics::lastNegativeResponse();
            WebConsoleHandler::log("[DTC] Negative response: " + dtcText);
            if (sendTelemetry != nullptr) sendTelemetry("DTC", "ACTIVE", "DTC", dtcText.c_str(), "", "ERROR");
        } else {
            dtcText = "TIMEOUT";
            WebConsoleHandler::log("[DTC] Timeout waiting for ISO-TP response");
            if (sendTelemetry != nullptr) sendTelemetry("DTC", "ACTIVE", "DTC", "N/A", "", "TIMEOUT");
        }
        Obd::Diagnostics::setDtc(dtcText.c_str());
        return;
    }

    char dtcList[64] = {0};
    lastCanMessageAt = nowMs;
    lastObdResponseAt = nowMs;
    canBusActive = true;
    const size_t dataOffset = response.length > 0 ? 1 : 0;
    Obd::decodeDtcList(response.bytes.data() + dataOffset,
                       response.length > dataOffset ? response.length - dataOffset : 0,
                       dtcList,
                       sizeof(dtcList));

    if (dtcList[0] == '\0') {
        dtcText = "Keine";
        Obd::Diagnostics::setDtc("Keine");
        WebConsoleHandler::log("[DTC] No active DTCs");
        if (sendTelemetry != nullptr) sendTelemetry("DTC", "ACTIVE", "DTC", "Keine", "", "OK");
    } else {
        dtcText = dtcList;
        Obd::Diagnostics::setDtc(dtcList);
        WebConsoleHandler::log("[DTC] Active codes: " + String(dtcList));
        if (sendTelemetry != nullptr) sendTelemetry("DTC", "ACTIVE", "DTC", dtcList, "", "WARN");
    }
}

void queryAndSendVin(uint32_t nowMs,
                     uint32_t& lastCanMessageAt,
                     uint32_t& lastObdResponseAt,
                     bool& canBusActive,
                     SenderCallbacks::SendTelemetry sendTelemetry) {
    const uint8_t payload[] = {read_VehicleInfo, read_VIN};
    IsoTp::Payload response{};

    if (!OBD2Handler::requestPayload(read_VehicleInfo, payload, sizeof(payload), read_VIN, response)) {
        if (OBD2Handler::lastResponseWasNegative()) {
            vinText = Obd::Diagnostics::lastNegativeResponse();
            WebConsoleHandler::log("[VIN] Negative response: " + vinText);
            if (sendTelemetry != nullptr) sendTelemetry("STATUS", "VIN", "VIN", vinText.c_str(), "", "ERROR");
        } else {
            vinText = "TIMEOUT";
            WebConsoleHandler::log("[VIN] Timeout waiting for ISO-TP response");
            if (sendTelemetry != nullptr) sendTelemetry("STATUS", "VIN", "VIN", "N/A", "", "TIMEOUT");
        }
        Obd::Diagnostics::setVin(vinText.c_str());
        return;
    }

    char vin[24] = {};
    if (Obd::decodeVin(response.bytes.data(), response.length, vin, sizeof(vin))) {
        lastCanMessageAt = nowMs;
        lastObdResponseAt = nowMs;
        canBusActive = true;
        vinText = vin;
        Obd::Diagnostics::setVin(vin);
        WebConsoleHandler::log("[VIN] " + String(vin));
        if (sendTelemetry != nullptr) sendTelemetry("STATUS", "VIN", "VIN", vin, "", "OK");
    } else {
        vinText = "INVALID";
        Obd::Diagnostics::setVin("INVALID");
        WebConsoleHandler::log("[VIN] Invalid Mode 09 PID 02 response");
        if (sendTelemetry != nullptr) sendTelemetry("STATUS", "VIN", "VIN", "N/A", "", "ERROR");
    }
}
} // namespace

namespace SenderObdScheduler {

void reset() {
    lastObdPollAt = 0;
    lastSupportedPidRefreshAt = 0;
    lastDtcQueryAt = 0;
    lastVinQueryAt = 0;
    supportedInitialized = false;
    buildPlanner();
    supportedPids01_20 = 0;
    supportedPids21_40 = 0;
    supportedPids41_60 = 0;
    dtcText = "--";
    vinText = "--";
}

void tick(uint32_t nowMs,
          uint32_t& lastCanMessageAt,
          uint32_t& lastObdResponseAt,
          bool& canBusActive,
          SenderCallbacks::SendTelemetry sendTelemetry,
          SenderCallbacks::SendStatus sendStatus) {
    if (!SenderConfig::EnableOBD2) return;
    if (nowMs - lastObdPollAt < SenderConfig::PollingRateMs) return;
    lastObdPollAt = nowMs;

    // CAN/OBD state reaches the display with the 1 s heartbeat; sending it here
    // (every 10 ms) flooded the radio link.
    canBusActive = (nowMs - lastCanMessageAt) <= SenderConfig::CanIdleTimeoutMs;

    // Without ECU (ignition off) every attempt blocks for the response timeout,
    // so retry the supported-PID query only every SupportedPidRetryMs.
    const uint32_t refreshInterval = supportedInitialized ? SenderConfig::SupportedPidRefreshMs
                                                          : SenderConfig::SupportedPidRetryMs;
    if (lastSupportedPidRefreshAt == 0 || nowMs - lastSupportedPidRefreshAt >= refreshInterval) {
        refreshSupportedPids(nowMs, lastCanMessageAt, lastObdResponseAt, canBusActive, sendStatus);
        lastSupportedPidRefreshAt = nowMs;
    }

    if (supportedInitialized) {
        // One live PID per tick keeps the loop responsive (web, ESP-NOW).
        byte pid = 0;
        if (planner.next(nowMs, pid)) {
            if (OBD2Handler::requestAndSendPID(pid)) {
                lastObdResponseAt = nowMs;
                lastCanMessageAt = nowMs;
                canBusActive = true;
            }
            planner.markPolled(pid, millis());
        }
        // ECU gone (ignition off): stop polling every PID into a timeout and
        // fall back to the slow supported-PID retry above.
        if (Obd::Diagnostics::timeoutStreak() >= SenderConfig::TimeoutsBeforeObdIdle) {
            supportedInitialized = false;
            lastSupportedPidRefreshAt = nowMs;
            WebConsoleHandler::log("[OBD] ECU antwortet nicht mehr, Abfrage pausiert");
        }
    }

    if (nowMs - lastDtcQueryAt >= SenderConfig::DtcQueryIntervalMs) {
        queryAndSendDtc(nowMs, lastCanMessageAt, lastObdResponseAt, canBusActive, sendTelemetry);
        lastDtcQueryAt = nowMs;
    }

    if (nowMs - lastVinQueryAt >= SenderConfig::VinQueryIntervalMs) {
        queryAndSendVin(nowMs, lastCanMessageAt, lastObdResponseAt, canBusActive, sendTelemetry);
        lastVinQueryAt = nowMs;
    }
}

bool supportedPidsInitialized() {
    return supportedInitialized;
}

const String& lastDtcText() {
    return dtcText;
}

const String& lastVinText() {
    return vinText;
}

} // namespace SenderObdScheduler
