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

namespace {
uint32_t lastObdPollAt = 0;
uint32_t lastSupportedPidRefreshAt = 0;
uint32_t lastDtcQueryAt = 0;
uint32_t lastVinQueryAt = 0;
bool supportedInitialized = false;
std::size_t nextPidIndex = 0;
uint32_t supportedPids01_20 = 0;
uint32_t supportedPids21_40 = 0;
uint32_t supportedPids41_60 = 0;
bool unsupportedPidReported[ObdConfig::ObdPidCount] = {false};
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
    nextPidIndex = 0;
    supportedPids01_20 = 0;
    supportedPids21_40 = 0;
    supportedPids41_60 = 0;
    dtcText = "--";
    vinText = "--";
    for (bool& reported : unsupportedPidReported) reported = false;
}

void tick(uint32_t nowMs,
          uint32_t& lastCanMessageAt,
          uint32_t& lastObdResponseAt,
          bool& canBusActive,
          SenderCallbacks::SendTelemetry sendTelemetry,
          SenderCallbacks::SendStatus sendStatus) {
    if (!SenderConfig::EnableOBD2) return;
    if (nowMs - lastObdPollAt < SenderConfig::ObdPollIntervalMs) return;
    lastObdPollAt = nowMs;

    canBusActive = (nowMs - lastCanMessageAt) <= SenderConfig::CanIdleTimeoutMs;
    if (sendStatus != nullptr) {
        sendStatus("CAN", canBusActive ? "ACTIVE" : "IDLE", canBusActive ? "OK" : "TIMEOUT");
    }

    if (!supportedInitialized ||
        nowMs - lastSupportedPidRefreshAt >= SenderConfig::SupportedPidRefreshMs) {
        refreshSupportedPids(nowMs, lastCanMessageAt, lastObdResponseAt, canBusActive, sendStatus);
        lastSupportedPidRefreshAt = nowMs;
    }

    if (supportedInitialized) {
        std::size_t queriedPids = 0;
        std::size_t inspectedPids = 0;
        while (inspectedPids < ObdConfig::ObdPidCount &&
               queriedPids < SenderConfig::MaxObdPidsPerTick) {
            const std::size_t pidIndex = nextPidIndex;
            nextPidIndex = (nextPidIndex + 1) % ObdConfig::ObdPidCount;
            ++inspectedPids;

            const byte pid = ObdConfig::RequestedPids[pidIndex];
            if (!isPidSupported(pid)) {
                if (!unsupportedPidReported[pidIndex] && sendTelemetry != nullptr) {
                    char pidHex[4];
                    snprintf(pidHex, sizeof(pidHex), "%02X", pid);
                    sendTelemetry("OBD", pidHex, getPIDName(pid), "N/A", "", "UNSUPPORTED");
                    unsupportedPidReported[pidIndex] = true;
                }
                continue;
            }

            if (OBD2Handler::requestAndSendPID(pid)) {
                lastObdResponseAt = nowMs;
                lastCanMessageAt = nowMs;
                canBusActive = true;
            }
            ++queriedPids;
        }
    } else if (sendStatus != nullptr) {
        sendStatus("OBD", "PID_SUPPORT_TIMEOUT", "WARN");
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
