#include "SenderCapabilityScanner.h"

#include <cstdio>
#include <cstring>

#include "CANHandler.h"
#include "CanRouterHub.h"
#include "CanSignalDiff.h"
#include "OBDHandler.h"
#include "ObdCapability.h"
#include "PidDecoder.h"
#include "UdsCapability.h"
#include "UdsClient.h"
#include "UdsDecoder.h"
#include "UdsTypes.h"
#include "WebConsoleHandler.h"
#include "config/ProjectConfig.h"
#include "config/SenderConfig.h"

namespace {

using Capabilities::CapabilityStatus;
using Capabilities::ObdPidCapability;
using Capabilities::EcuCapability;
using Capabilities::UdsDidCapability;

constexpr uint32_t kScannerStepIntervalMs = 80;
constexpr uint32_t kUdsPendingTotalTimeoutMs = 4000;
constexpr std::size_t kObdResultCapacity = Capabilities::MaxObdPidResults;
constexpr std::size_t kEcuCapacity = Capabilities::MaxEcuResults;
constexpr std::size_t kDidCapacity = Capabilities::MaxDidResults;
constexpr std::size_t kSnifferFrameCapacity = 48;

SenderCapabilityScanner::ActiveScan scan = SenderCapabilityScanner::ActiveScan::Idle;
Capabilities::ScanState state = Capabilities::ScanState::Idle;
uint32_t startedAt = 0;
uint32_t lastStepAt = 0;
uint32_t completedAt = 0;
String lastMessage = "Bereit";

ObdPidCapability obdResults[kObdResultCapacity]{};
std::size_t obdResultCount = 0;
std::size_t obdStepIndex = 0;
uint32_t mask01_20 = 0;
uint32_t mask21_40 = 0;
uint32_t mask41_60 = 0;
uint32_t mask61_80 = 0;
bool maskReady = false;

EcuCapability ecuResults[kEcuCapacity]{};
std::size_t ecuResultCount = 0;
std::size_t ecuStepIndex = 0;

UdsDidCapability didResults[kDidCapacity]{};
std::size_t didResultCount = 0;
std::size_t didStepIndex = 0;
bool didResultsInitialized = false;
Uds::Client udsClient;

uint32_t canSnifferFrames = 0;
uint32_t canSnifferDroppedFrames = 0;
bool canSnifferRegistered = false;
Capabilities::CanFrameSample snifferFrames[kSnifferFrameCapacity]{};
std::size_t snifferFrameCount = 0;
Capabilities::CanSignalCandidate snifferCandidates[Capabilities::MaxCanSignalCandidates]{};
std::size_t snifferCandidateCount = 0;

Capabilities::CanFrameSample sampleFromFrame(const CanRouting::CanFrame& frame) {
    Capabilities::CanFrameSample sample{};
    sample.canId = frame.id;
    sample.length = frame.length > 8 ? 8 : frame.length;
    for (uint8_t index = 0; index < sample.length; ++index) {
        sample.data[index] = frame.data[index];
    }
    return sample;
}

int findSnifferFrame(uint32_t canId) {
    for (std::size_t index = 0; index < snifferFrameCount; ++index) {
        if (snifferFrames[index].canId == canId) return static_cast<int>(index);
    }
    return -1;
}

int findSnifferCandidate(const Capabilities::CanSignalCandidate& candidate) {
    for (std::size_t index = 0; index < snifferCandidateCount; ++index) {
        if (snifferCandidates[index].canId == candidate.canId &&
            snifferCandidates[index].byteIndex == candidate.byteIndex &&
            snifferCandidates[index].changedBitMask == candidate.changedBitMask) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

void mergeSnifferCandidate(const Capabilities::CanSignalCandidate& candidate) {
    const int existing = findSnifferCandidate(candidate);
    if (existing >= 0) {
        Capabilities::CanSignalCandidate& target = snifferCandidates[existing];
        target.afterValue = candidate.afterValue;
        if (target.changeCount < UINT16_MAX) ++target.changeCount;
        return;
    }

    if (snifferCandidateCount >= Capabilities::MaxCanSignalCandidates) return;
    snifferCandidates[snifferCandidateCount] = candidate;
    snifferCandidates[snifferCandidateCount].changeCount = 1;
    ++snifferCandidateCount;
}

class CapabilityCanListener final : public CanRouting::CanFrameListener {
public:
    void onCanFrame(const CanRouting::CanFrame& frame) override {
        if (scan != SenderCapabilityScanner::ActiveScan::CanSniffer ||
            state != Capabilities::ScanState::Running) {
            return;
        }

        ++canSnifferFrames;
        const Capabilities::CanFrameSample current = sampleFromFrame(frame);
        const int existing = findSnifferFrame(current.canId);
        if (existing < 0) {
            if (snifferFrameCount < kSnifferFrameCapacity) {
                snifferFrames[snifferFrameCount++] = current;
            } else {
                ++canSnifferDroppedFrames;
            }
            return;
        }

        Capabilities::CanSignalCandidate diff[8]{};
        const std::size_t diffCount = Capabilities::diffCanFrames(snifferFrames[existing], current, diff, 8);
        for (std::size_t index = 0; index < diffCount; ++index) {
            mergeSnifferCandidate(diff[index]);
        }
        snifferFrames[existing] = current;
    }
};

CapabilityCanListener canListener;

String jsonEscape(const String& value) {
    String escaped;
    escaped.reserve(value.length() + 4);
    for (size_t i = 0; i < value.length(); ++i) {
        const char c = value[i];
        switch (c) {
            case '\\': escaped += "\\\\"; break;
            case '"': escaped += "\\\""; break;
            case '\n': escaped += "\\n"; break;
            case '\r': break;
            case '\t': escaped += "\\t"; break;
            default:
                if (static_cast<uint8_t>(c) < 0x20) escaped += ' ';
                else escaped += c;
                break;
        }
    }
    return escaped;
}

const char* scanName(SenderCapabilityScanner::ActiveScan value) {
    switch (value) {
        case SenderCapabilityScanner::ActiveScan::Idle: return "idle";
        case SenderCapabilityScanner::ActiveScan::ObdPids: return "obd";
        case SenderCapabilityScanner::ActiveScan::Uds: return "uds";
        case SenderCapabilityScanner::ActiveScan::CanSniffer: return "can_sniffer";
    }
    return "unknown";
}

const char* stateName(Capabilities::ScanState value) {
    switch (value) {
        case Capabilities::ScanState::Idle: return "IDLE";
        case Capabilities::ScanState::Running: return "RUNNING";
        case Capabilities::ScanState::Stopped: return "STOPPED";
        case Capabilities::ScanState::Complete: return "COMPLETED";
        case Capabilities::ScanState::Error: return "FAILED";
    }
    return "UNKNOWN";
}

uint32_t bytesToMask(const uint8_t* data, uint8_t length) {
    if (data == nullptr || length < 4) return 0;
    return (static_cast<uint32_t>(data[0]) << 24U) |
           (static_cast<uint32_t>(data[1]) << 16U) |
           (static_cast<uint32_t>(data[2]) << 8U) |
           static_cast<uint32_t>(data[3]);
}

bool queryMask(uint8_t rangePid, uint32_t& outMask) {
    uint8_t responseData[8]{};
    uint8_t responseLen = 0;
    if (!OBD2Handler::sendRequest(0x01, rangePid)) return false;
    if (!OBD2Handler::receiveResponse(0x01, rangePid, responseData, responseLen)) return false;
    outMask = bytesToMask(responseData, responseLen);
    return responseLen >= 4;
}

void initializeObdResults() {
    obdResultCount = 0;
    obdStepIndex = 0;
    const auto* descriptors = Capabilities::recommendedObdPids(obdResultCount);
    if (obdResultCount > kObdResultCapacity) obdResultCount = kObdResultCapacity;
    for (std::size_t i = 0; i < obdResultCount; ++i) {
        ObdPidCapability& result = obdResults[i];
        result = ObdPidCapability{};
        result.pid = descriptors[i].pid;
        std::snprintf(result.name, sizeof(result.name), "%s", descriptors[i].name);
        std::snprintf(result.unit, sizeof(result.unit), "%s", descriptors[i].unit);
        std::snprintf(result.status, sizeof(result.status), "%s", "PENDING");
    }
}

void initializeUdsResults() {
    ecuResultCount = 0;
    ecuStepIndex = 0;
    didResultCount = 0;
    didStepIndex = 0;
    didResultsInitialized = false;

    for (uint32_t requestId = Capabilities::UdsFirstRequestId;
         requestId <= Capabilities::UdsLastRequestId && ecuResultCount < kEcuCapacity;
         ++requestId) {
        EcuCapability& ecu = ecuResults[ecuResultCount++];
        ecu = EcuCapability{};
        ecu.requestId = requestId;
        ecu.responseId = Capabilities::udsResponseIdForRequestId(requestId);
        std::snprintf(ecu.ecuName, sizeof(ecu.ecuName), "ECU 0x%03lX", static_cast<unsigned long>(requestId));
    }
}

void initializeDidResultsForReachableEcus() {
    didResultCount = 0;
    didStepIndex = 0;
    didResultsInitialized = true;

    uint8_t didProbeCount = 0;
    const auto* dids = Capabilities::udsDidProbes(didProbeCount);
    for (std::size_t ecuIndex = 0; ecuIndex < ecuResultCount; ++ecuIndex) {
        const EcuCapability& ecu = ecuResults[ecuIndex];
        if (!ecu.reachable && !ecu.supportsUds) continue;

        for (uint8_t didIndex = 0; didIndex < didProbeCount && didResultCount < kDidCapacity; ++didIndex) {
            UdsDidCapability& did = didResults[didResultCount++];
            did = UdsDidCapability{};
            did.requestId = ecu.requestId;
            did.responseId = ecu.responseId;
            did.did = dids[didIndex].did;
            std::snprintf(did.name, sizeof(did.name), "%s", dids[didIndex].name);
            std::snprintf(did.status, sizeof(did.status), "%s", "PENDING");
        }
    }
}

void complete(Capabilities::ScanState finalState, const String& message) {
    state = finalState;
    completedAt = millis();
    lastMessage = message;
    if (finalState == Capabilities::ScanState::Complete) {
        WebConsoleHandler::log("[Capabilities] Scan completed: " + message);
    } else if (finalState == Capabilities::ScanState::Error) {
        WebConsoleHandler::log("[Capabilities] Scan failed: " + message);
    } else {
        WebConsoleHandler::log("[Capabilities] Scan stopped: " + message);
    }
    if (canSnifferRegistered) {
        CanRouting::unregisterListener(canListener);
        canSnifferRegistered = false;
    }
    scan = SenderCapabilityScanner::ActiveScan::Idle;
}

void runObdStep() {
    if (!maskReady) {
        WebConsoleHandler::log("[Capabilities] Querying OBD supported PID masks");
        const bool firstOk = queryMask(0x00, mask01_20);
        const bool secondOk = firstOk && Capabilities::maskAdvertisesNextRange(0x00, mask01_20)
                            ? queryMask(0x20, mask21_40)
                            : true;
        const bool thirdOk = secondOk && Capabilities::maskAdvertisesNextRange(0x20, mask21_40)
                           ? queryMask(0x40, mask41_60)
                           : true;
        const bool fourthOk = thirdOk && Capabilities::maskAdvertisesNextRange(0x40, mask41_60)
                            ? queryMask(0x60, mask61_80)
                            : true;
        maskReady = firstOk && secondOk && thirdOk && fourthOk;
        if (!maskReady) {
            complete(Capabilities::ScanState::Error, "PID-Masken konnten nicht gelesen werden");
        }
        return;
    }

    if (obdStepIndex >= obdResultCount) {
        complete(Capabilities::ScanState::Complete, "OBD PID Scan abgeschlossen");
        return;
    }

    ObdPidCapability& result = obdResults[obdStepIndex++];
    result.supportedByMask = Capabilities::isPidSupportedByMask(result.pid, mask01_20, mask21_40, mask41_60, mask61_80);
    if (!result.supportedByMask) {
        const CapabilityStatus status = CapabilityStatus::Unsupported;
        std::snprintf(result.status, sizeof(result.status), "%s", Capabilities::statusText(status));
        return;
    }

    const uint32_t requestStarted = millis();
    uint8_t responseData[8]{};
    uint8_t responseLen = 0;
    CapabilityStatus status = CapabilityStatus::Unknown;
    if (!OBD2Handler::sendRequest(0x01, result.pid)) {
        status = CapabilityStatus::SendFailed;
    } else if (!OBD2Handler::receiveResponse(0x01, result.pid, responseData, responseLen)) {
        status = OBD2Handler::lastResponseWasNegative()
            ? CapabilityStatus::NegativeResponse
            : CapabilityStatus::Timeout;
    } else {
        result.responded = true;
        const Obd::PidValue decoded = Obd::decodePid(result.pid, responseData, responseLen);
        result.validDecoded = decoded.valid;
        result.responseTimeMs = millis() - requestStarted;
        if (decoded.valid) {
            status = CapabilityStatus::Ok;
            std::snprintf(result.exampleValue, sizeof(result.exampleValue), "%.2f", static_cast<double>(decoded.value));
            std::snprintf(result.unit, sizeof(result.unit), "%s", decoded.unit);
        } else {
            status = CapabilityStatus::DecodeError;
            std::snprintf(result.exampleValue, sizeof(result.exampleValue), "%s", "raw");
        }
    }
    std::snprintf(result.status, sizeof(result.status), "%s", Capabilities::statusText(status));
}

bool requestTesterPresentWithPending(Uds::Response& response) {
    const uint32_t started = millis();
    uint32_t pendingCount = 0;
    do {
        if (udsClient.testerPresent(response)) return true;
        if (!response.negative || !Capabilities::isUdsResponsePending(response.negativeCode)) return false;
        ++pendingCount;
        delay(50);
    } while (Capabilities::shouldContinueWaitingForPending(started, millis(), kUdsPendingTotalTimeoutMs));
    (void)pendingCount;
    return false;
}

bool requestDidWithPending(uint16_t did, Uds::Response& response) {
    const uint32_t started = millis();
    uint32_t pendingCount = 0;
    do {
        if (udsClient.readDataByIdentifier(did, response)) return true;
        if (!response.negative || !Capabilities::isUdsResponsePending(response.negativeCode)) return false;
        ++pendingCount;
        delay(50);
    } while (Capabilities::shouldContinueWaitingForPending(started, millis(), kUdsPendingTotalTimeoutMs));
    (void)pendingCount;
    return false;
}

void runUdsStep() {
    if (ecuStepIndex < ecuResultCount) {
        EcuCapability& ecu = ecuResults[ecuStepIndex++];
        udsClient.setRequestId(ecu.requestId);
        Uds::Response response{};
        const bool ok = requestTesterPresentWithPending(response);
        ecu.reachable = ok || response.negative;
        ecu.supportsUds = ok || (response.negative && response.negativeCode != 0);
        if (response.responseId != 0) ecu.responseId = response.responseId;
        if (ok) {
            std::snprintf(ecu.ecuName, sizeof(ecu.ecuName), "UDS ECU 0x%03lX", static_cast<unsigned long>(ecu.requestId));
        }
        return;
    }

    if (!didResultsInitialized) {
        initializeDidResultsForReachableEcus();
        if (didResultCount == 0) {
            complete(Capabilities::ScanState::Complete, "UDS Scan abgeschlossen, keine erreichbare ECU");
            return;
        }
    }

    if (didStepIndex >= didResultCount) {
        complete(Capabilities::ScanState::Complete, "UDS Scan abgeschlossen");
        return;
    }

    UdsDidCapability& did = didResults[didStepIndex++];
    udsClient.setRequestId(did.requestId);
    Uds::Response response{};
    const bool ok = requestDidWithPending(did.did, response);
    did.responseId = response.responseId != 0 ? response.responseId : did.responseId;
    CapabilityStatus status = CapabilityStatus::Unknown;
    if (ok) {
        did.responded = true;
        char decoded[sizeof(did.value)]{};
        if (Uds::decodeAsciiDid(response.data, response.length, did.did, decoded, sizeof(decoded))) {
            std::snprintf(did.value, sizeof(did.value), "%s", decoded);
            status = CapabilityStatus::Ok;
        } else {
            std::snprintf(did.value, sizeof(did.value), "len=%u", static_cast<unsigned>(response.length));
            status = CapabilityStatus::Ok;
        }
    } else if (response.negative) {
        if (Capabilities::isUdsResponsePending(response.negativeCode)) {
            status = CapabilityStatus::Timeout;
            did.pendingSeen = true;
            ++did.pendingCount;
            std::snprintf(did.value, sizeof(did.value), "0x78 ResponsePending timeout");
        } else {
            status = Capabilities::statusForNegativeResponse(response.negativeCode);
            std::snprintf(did.value, sizeof(did.value), "NRC 0x%02X", response.negativeCode);
        }
    } else {
        status = CapabilityStatus::Timeout;
    }
    std::snprintf(did.status, sizeof(did.status), "%s", Capabilities::statusText(status));
}

void appendObdJson(String& json) {
    json += "\"obd\":{\"masks\":{\"01_20\":\"0x" + String(mask01_20, HEX) +
            "\",\"21_40\":\"0x" + String(mask21_40, HEX) +
            "\",\"41_60\":\"0x" + String(mask41_60, HEX) +
            "\",\"61_80\":\"0x" + String(mask61_80, HEX) + "\"},\"pids\":[";
    for (std::size_t i = 0; i < obdResultCount; ++i) {
        const auto& result = obdResults[i];
        if (i > 0) json += ",";
        json += "{";
        json += "\"pid\":\"0x" + String(result.pid, HEX) + "\",";
        json += "\"name\":\"" + jsonEscape(result.name) + "\",";
        json += "\"unit\":\"" + jsonEscape(result.unit) + "\",";
        json += "\"supportedByMask\":" + String(result.supportedByMask ? "true" : "false") + ",";
        json += "\"responded\":" + String(result.responded ? "true" : "false") + ",";
        json += "\"validDecoded\":" + String(result.validDecoded ? "true" : "false") + ",";
        json += "\"responseTimeMs\":" + String(result.responseTimeMs) + ",";
        json += "\"exampleValue\":\"" + jsonEscape(result.exampleValue) + "\",";
        json += "\"status\":\"" + jsonEscape(result.status) + "\"";
        json += "}";
    }
    json += "]}";
}

void appendUdsJson(String& json) {
    json += "\"uds\":{\"ecus\":[";
    for (std::size_t i = 0; i < ecuResultCount; ++i) {
        const auto& ecu = ecuResults[i];
        if (i > 0) json += ",";
        json += "{";
        json += "\"requestId\":\"0x" + String(ecu.requestId, HEX) + "\",";
        json += "\"responseId\":\"0x" + String(ecu.responseId, HEX) + "\",";
        json += "\"reachable\":" + String(ecu.reachable ? "true" : "false") + ",";
        json += "\"supportsUds\":" + String(ecu.supportsUds ? "true" : "false") + ",";
        json += "\"supportsObd\":" + String(ecu.supportsObd ? "true" : "false") + ",";
        json += "\"vin\":\"" + jsonEscape(ecu.vin) + "\",";
        json += "\"ecuName\":\"" + jsonEscape(ecu.ecuName) + "\"";
        json += "}";
    }
    json += "],\"dids\":[";
    for (std::size_t i = 0; i < didResultCount; ++i) {
        const auto& did = didResults[i];
        if (i > 0) json += ",";
        json += "{";
        json += "\"ecu\":\"0x" + String(did.requestId, HEX) + "\",";
        json += "\"did\":\"0x" + String(did.did, HEX) + "\",";
        json += "\"name\":\"" + jsonEscape(did.name) + "\",";
        json += "\"value\":\"" + jsonEscape(did.value) + "\",";
        json += "\"status\":\"" + jsonEscape(did.status) + "\"";
        json += "}";
    }
    json += "]}";
}

void appendCanSnifferJson(String& json) {
    json += "\"canSniffer\":{\"active\":" + String(scan == SenderCapabilityScanner::ActiveScan::CanSniffer ? "true" : "false") +
            ",\"frames\":" + String(canSnifferFrames) +
            ",\"droppedFrames\":" + String(canSnifferDroppedFrames) +
            ",\"knownIds\":" + String(snifferFrameCount) +
            ",\"candidateCount\":" + String(snifferCandidateCount) +
            ",\"candidates\":[";
    for (std::size_t i = 0; i < snifferCandidateCount; ++i) {
        const auto& candidate = snifferCandidates[i];
        if (i > 0) json += ",";
        json += "{";
        json += "\"canId\":\"0x" + String(candidate.canId, HEX) + "\",";
        json += "\"byteIndex\":" + String(candidate.byteIndex) + ",";
        json += "\"before\":\"0x" + String(candidate.beforeValue, HEX) + "\",";
        json += "\"after\":\"0x" + String(candidate.afterValue, HEX) + "\",";
        json += "\"changedBitMask\":\"0x" + String(candidate.changedBitMask, HEX) + "\",";
        json += "\"changeCount\":" + String(candidate.changeCount);
        json += "}";
    }
    json += "]}";
}

} // namespace

namespace SenderCapabilityScanner {

void reset() {
    if (canSnifferRegistered) {
        CanRouting::unregisterListener(canListener);
        canSnifferRegistered = false;
    }
    scan = ActiveScan::Idle;
    state = Capabilities::ScanState::Idle;
    startedAt = 0;
    lastStepAt = 0;
    completedAt = 0;
    lastMessage = "Bereit";
    mask01_20 = mask21_40 = mask41_60 = mask61_80 = 0;
    maskReady = false;
    obdResultCount = 0;
    obdStepIndex = 0;
    ecuResultCount = 0;
    ecuStepIndex = 0;
    didResultCount = 0;
    didStepIndex = 0;
    didResultsInitialized = false;
    canSnifferFrames = 0;
    canSnifferDroppedFrames = 0;
    snifferFrameCount = 0;
    snifferCandidateCount = 0;
    for (auto& frame : snifferFrames) frame = Capabilities::CanFrameSample{};
    for (auto& candidate : snifferCandidates) candidate = Capabilities::CanSignalCandidate{};
}

bool active() {
    return scan != ActiveScan::Idle && state == Capabilities::ScanState::Running;
}

ActiveScan activeScan() {
    return scan;
}

void startObdPidScan() {
    reset();
    scan = ActiveScan::ObdPids;
    state = Capabilities::ScanState::Running;
    startedAt = millis();
    initializeObdResults();
    lastMessage = "OBD PID Scan laeuft";
    WebConsoleHandler::log("[Capabilities] OBD PID scan started");
}

void startUdsScan() {
    reset();
    scan = ActiveScan::Uds;
    state = Capabilities::ScanState::Running;
    startedAt = millis();
    initializeUdsResults();
    lastMessage = "UDS Scan laeuft";
    WebConsoleHandler::log("[Capabilities] UDS scan started");
}

void startCanSniffer() {
    reset();
    scan = ActiveScan::CanSniffer;
    state = Capabilities::ScanState::Running;
    startedAt = millis();
    canSnifferRegistered = CanRouting::registerListener(canListener);
    lastMessage = canSnifferRegistered
        ? "CAN Sniffer aktiv (CanRouter Listener)"
        : "CAN Sniffer konnte nicht registriert werden";
    WebConsoleHandler::log(canSnifferRegistered
        ? "[Capabilities] Passive CAN sniffer registered on CanRouter"
        : "[Capabilities] Passive CAN sniffer registration failed");
    if (!canSnifferRegistered) {
        complete(Capabilities::ScanState::Error, lastMessage);
    }
}

void resetCanSnifferBaseline() {
    canSnifferFrames = 0;
    canSnifferDroppedFrames = 0;
    snifferFrameCount = 0;
    snifferCandidateCount = 0;
    for (auto& frame : snifferFrames) frame = Capabilities::CanFrameSample{};
    for (auto& candidate : snifferCandidates) candidate = Capabilities::CanSignalCandidate{};
    lastMessage = activeScan() == ActiveScan::CanSniffer
        ? "CAN Sniffer Baseline neu gesetzt"
        : "CAN Sniffer Baseline geloescht";
    WebConsoleHandler::log("[Capabilities] CAN sniffer baseline reset");
}

void stop() {
    if (scan == ActiveScan::Idle) {
        state = Capabilities::ScanState::Idle;
        lastMessage = "Kein Scan aktiv";
        return;
    }
    complete(Capabilities::ScanState::Stopped, "Scan manuell gestoppt");
}

void tick(uint32_t nowMs) {
    if (!active()) return;
    if (nowMs - lastStepAt < kScannerStepIntervalMs) return;
    lastStepAt = nowMs;

    switch (scan) {
        case ActiveScan::ObdPids:
            runObdStep();
            break;
        case ActiveScan::Uds:
            runUdsStep();
            break;
        case ActiveScan::CanSniffer:
            // CAN frames are collected asynchronously through CanRouter when
            // CANHandler::processIncoming() drains the TWAI RX queue.
            break;
        case ActiveScan::Idle:
            break;
    }
}

String statusJson() {
    String json;
    json.reserve(4096);
    json += "{";
    json += "\"firmware\":\"" + jsonEscape(ProjectConfig::FirmwareVersion) + "\",";
    json += "\"target\":\"" + jsonEscape(ProjectConfig::TargetName) + "\",";
    json += "\"active\":" + String(active() ? "true" : "false") + ",";
    json += "\"scan\":\"" + String(scanName(scan)) + "\",";
    json += "\"state\":\"" + String(stateName(state)) + "\",";
    json += "\"message\":\"" + jsonEscape(lastMessage) + "\",";
    json += "\"startedAt\":" + String(startedAt) + ",";
    json += "\"completedAt\":" + String(completedAt) + ",";
    json += "\"canSnifferFrames\":" + String(canSnifferFrames) + ",";
    appendObdJson(json);
    json += ",";
    appendUdsJson(json);
    json += ",";
    appendCanSnifferJson(json);
    json += "}";
    return json;
}

String exportJson() {
    String json;
    json.reserve(6144);
    json += "{";
    json += "\"firmwareVersion\":\"" + jsonEscape(ProjectConfig::FirmwareVersion) + "\",";
    json += "\"target\":\"" + jsonEscape(ProjectConfig::TargetName) + "\",";
    json += "\"protocol\":" + String(ProjectConfig::ProtocolVersion) + ",";
    json += "\"timestampMs\":" + String(millis()) + ",";
    json += "\"scan\":\"" + String(scanName(scan)) + "\",";
    json += "\"state\":\"" + String(stateName(state)) + "\",";
    json += "\"message\":\"" + jsonEscape(lastMessage) + "\",";
    appendObdJson(json);
    json += ",";
    appendUdsJson(json);
    json += ",";
    appendCanSnifferJson(json);
    json += "}";
    return json;
}

} // namespace SenderCapabilityScanner
