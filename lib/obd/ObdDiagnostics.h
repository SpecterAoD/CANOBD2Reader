#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "IsoTpTypes.h"

namespace Obd {

/// Runtime-only OBD diagnostic counters and last-message snapshots.
///
/// The sender web console uses this state to distinguish "OBD is not queried",
/// "OBD is queried but the ECU is silent" and "the ECU answers with a negative
/// response". Keeping this logic outside SenderApp makes it native-testable and
/// avoids another cluster of unrelated global variables.
class Diagnostics {
public:
    static void reset() {
        requestCount_ = 0;
        sendFailureCount_ = 0;
        timeoutCount_ = 0;
        validResponseCount_ = 0;
        negativeResponseCount_ = 0;
        timeoutStreak_ = 0;
        supportedPidsReady_ = false;
        physicalFallbackActive_ = false;
        requestCanId_ = IsoTp::FunctionalRequestId;
        supportedPidMasks_[0] = 0;
        supportedPidMasks_[1] = 0;
        supportedPidMasks_[2] = 0;
        copyText(lastRequest_, sizeof(lastRequest_), "--");
        copyText(lastEcuResponse_, sizeof(lastEcuResponse_), "--");
        copyText(lastNegativeResponse_, sizeof(lastNegativeResponse_), "--");
        copyText(lastVin_, sizeof(lastVin_), "--");
        copyText(lastDtc_, sizeof(lastDtc_), "--");
    }

    static void recordRequest(uint8_t mode, const uint8_t* payload, std::size_t payloadLength, uint32_t canId) {
        ++requestCount_;
        requestCanId_ = canId;
        char hex[48] = {};
        std::size_t offset = 0;
        for (std::size_t index = 0; index < payloadLength && offset + 4 < sizeof(hex); ++index) {
            offset += static_cast<std::size_t>(
                std::snprintf(hex + offset, sizeof(hex) - offset, "%s%02X", index == 0 ? "" : " ", payload[index]));
        }
        std::snprintf(lastRequest_, sizeof(lastRequest_),
                      "id=0x%03lX mode=0x%02X payload=[%s]",
                      static_cast<unsigned long>(canId),
                      mode,
                      payloadLength > 0 ? hex : "--");
    }

    static void recordRequest(uint8_t mode, uint8_t pid, uint32_t canId) {
        const uint8_t payload[] = {mode, pid};
        recordRequest(mode, payload, sizeof(payload), canId);
    }

    static void recordSendFailure() {
        ++sendFailureCount_;
        copyText(lastEcuResponse_, sizeof(lastEcuResponse_), "SEND_FAIL");
    }

    static void recordTimeout() {
        ++timeoutCount_;
        ++timeoutStreak_;
        copyText(lastEcuResponse_, sizeof(lastEcuResponse_), "TIMEOUT");
    }

    static void recordValidResponse(uint32_t responseId, const uint8_t* payload, std::size_t payloadLength) {
        ++validResponseCount_;
        timeoutStreak_ = 0;

        std::size_t offset = static_cast<std::size_t>(
            std::snprintf(lastEcuResponse_, sizeof(lastEcuResponse_),
                          "id=0x%03lX len=%u data=",
                          static_cast<unsigned long>(responseId),
                          static_cast<unsigned>(payloadLength)));
        for (std::size_t index = 0; index < payloadLength && offset + 4 < sizeof(lastEcuResponse_); ++index) {
            offset += static_cast<std::size_t>(
                std::snprintf(lastEcuResponse_ + offset,
                              sizeof(lastEcuResponse_) - offset,
                              "%s%02X",
                              index == 0 ? "" : " ",
                              payload[index]));
        }
    }

    static void recordNegativeResponse(uint8_t service, uint8_t nrc) {
        ++negativeResponseCount_;
        timeoutStreak_ = 0;
        std::snprintf(lastNegativeResponse_, sizeof(lastNegativeResponse_),
                      "service=0x%02X nrc=0x%02X %s",
                      service,
                      nrc,
                      negativeResponseDescription(nrc));
        copyText(lastEcuResponse_, sizeof(lastEcuResponse_), lastNegativeResponse_);
    }

    static void setSupportedPidMasks(uint32_t mask01_20, uint32_t mask21_40, uint32_t mask41_60, bool ready) {
        supportedPidMasks_[0] = mask01_20;
        supportedPidMasks_[1] = mask21_40;
        supportedPidMasks_[2] = mask41_60;
        supportedPidsReady_ = ready;
    }

    static void setVin(const char* vin) {
        copyText(lastVin_, sizeof(lastVin_), vin != nullptr && vin[0] != '\0' ? vin : "--");
    }

    static void setDtc(const char* dtc) {
        copyText(lastDtc_, sizeof(lastDtc_), dtc != nullptr && dtc[0] != '\0' ? dtc : "--");
    }

    // Triggers only when consecutive timeouts have reached the threshold and a
    // fallback has not already been activated. The streak guard prevents a single
    // transient timeout from forcing a premature switch to physical addressing.
    static bool shouldSwitchToPhysicalFallback(uint8_t threshold) {
        return !physicalFallbackActive_ && threshold > 0 && timeoutStreak_ >= threshold;
    }

    // Switching to physical addressing changes the CAN request ID from the
    // OBD-II functional broadcast (0x7DF) to a unicast address (0x7E0). The
    // change persists for the session; there is no automatic revert because
    // reverting would restart the timeout streak cycle on vehicles that never
    // supported functional addressing.
    static void setPhysicalFallbackActive(bool active) {
        physicalFallbackActive_ = active;
        requestCanId_ = active ? IsoTp::PhysicalRequestId : IsoTp::FunctionalRequestId;
    }

    static uint32_t requestCount() { return requestCount_; }
    static uint32_t sendFailureCount() { return sendFailureCount_; }
    static uint32_t timeoutCount() { return timeoutCount_; }
    static uint32_t validResponseCount() { return validResponseCount_; }
    static uint32_t negativeResponseCount() { return negativeResponseCount_; }
    static uint32_t timeoutStreak() { return timeoutStreak_; }
    static bool supportedPidsReady() { return supportedPidsReady_; }
    static bool physicalFallbackActive() { return physicalFallbackActive_; }
    static uint32_t requestCanId() { return requestCanId_; }
    static uint32_t supportedPidMask(std::size_t index) { return index < 3 ? supportedPidMasks_[index] : 0; }
    static const char* lastRequest() { return lastRequest_; }
    static const char* lastEcuResponse() { return lastEcuResponse_; }
    static const char* lastNegativeResponse() { return lastNegativeResponse_; }
    static const char* lastVin() { return lastVin_; }
    static const char* lastDtc() { return lastDtc_; }

    static const char* negativeResponseDescription(uint8_t nrc) {
        switch (nrc) {
            case 0x10: return "GeneralReject";
            case 0x11: return "ServiceNotSupported";
            case 0x12: return "SubFunctionNotSupported";
            case 0x13: return "IncorrectMessageLength";
            case 0x21: return "BusyRepeatRequest";
            case 0x22: return "ConditionsNotCorrect";
            case 0x31: return "RequestOutOfRange";
            case 0x33: return "SecurityAccessDenied";
            case 0x78: return "ResponsePending";
            default: return "UnknownNRC";
        }
    }

private:
    static void copyText(char* target, std::size_t targetSize, const char* source) {
        if (target == nullptr || targetSize == 0) return;
        if (source == nullptr) source = "";
        std::snprintf(target, targetSize, "%s", source);
    }

    static inline uint32_t requestCount_ = 0;
    static inline uint32_t sendFailureCount_ = 0;
    static inline uint32_t timeoutCount_ = 0;
    static inline uint32_t validResponseCount_ = 0;
    static inline uint32_t negativeResponseCount_ = 0;
    static inline uint32_t timeoutStreak_ = 0;
    static inline bool supportedPidsReady_ = false;
    static inline bool physicalFallbackActive_ = false;
    static inline uint32_t requestCanId_ = IsoTp::FunctionalRequestId;
    static inline uint32_t supportedPidMasks_[3] = {};
    static inline char lastRequest_[96] = "--";
    static inline char lastEcuResponse_[128] = "--";
    static inline char lastNegativeResponse_[96] = "--";
    static inline char lastVin_[24] = "--";
    static inline char lastDtc_[96] = "--";
};

} // namespace Obd
