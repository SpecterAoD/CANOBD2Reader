#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "UdsTypes.h"

namespace Uds {

class Diagnostics {
public:
    static void reset() {
        requestCount_ = 0;
        sendFailureCount_ = 0;
        timeoutCount_ = 0;
        positiveResponseCount_ = 0;
        negativeResponseCount_ = 0;
        available_ = false;
        copyText(lastRequest_, sizeof(lastRequest_), "--");
        copyText(lastResponse_, sizeof(lastResponse_), "--");
        copyText(lastNegativeResponse_, sizeof(lastNegativeResponse_), "--");
        copyText(lastDid_, sizeof(lastDid_), "--");
        copyText(lastDtcSummary_, sizeof(lastDtcSummary_), "--");
    }

    static void recordRequest(Service service, const uint8_t* payload, std::size_t payloadLength, uint32_t canId) {
        ++requestCount_;
        char bytes[48] = {};
        std::size_t offset = 0;
        for (std::size_t index = 0; index < payloadLength && offset + 4 < sizeof(bytes); ++index) {
            offset += static_cast<std::size_t>(
                std::snprintf(bytes + offset, sizeof(bytes) - offset, "%s%02X", index == 0 ? "" : " ", payload[index]));
        }
        std::snprintf(lastRequest_, sizeof(lastRequest_),
                      "id=0x%03lX service=0x%02X %s payload=[%s]",
                      static_cast<unsigned long>(canId),
                      serviceId(service),
                      serviceName(service),
                      payloadLength > 0 ? bytes : "--");
    }

    static void recordSendFailure() {
        ++sendFailureCount_;
        copyText(lastResponse_, sizeof(lastResponse_), "SEND_FAIL");
    }

    static void recordTimeout() {
        ++timeoutCount_;
        copyText(lastResponse_, sizeof(lastResponse_), "TIMEOUT");
    }

    static void recordPositiveResponse(uint32_t responseId, const uint8_t* payload, std::size_t payloadLength) {
        ++positiveResponseCount_;
        available_ = true;
        std::size_t offset = static_cast<std::size_t>(
            std::snprintf(lastResponse_, sizeof(lastResponse_),
                          "id=0x%03lX len=%u data=",
                          static_cast<unsigned long>(responseId),
                          static_cast<unsigned>(payloadLength)));
        for (std::size_t index = 0; index < payloadLength && offset + 4 < sizeof(lastResponse_); ++index) {
            offset += static_cast<std::size_t>(
                std::snprintf(lastResponse_ + offset,
                              sizeof(lastResponse_) - offset,
                              "%s%02X",
                              index == 0 ? "" : " ",
                              payload[index]));
        }
    }

    static void recordNegativeResponse(uint8_t service, uint8_t nrc) {
        ++negativeResponseCount_;
        std::snprintf(lastNegativeResponse_, sizeof(lastNegativeResponse_),
                      "service=0x%02X nrc=0x%02X %s",
                      service,
                      nrc,
                      negativeResponseDescription(nrc));
        copyText(lastResponse_, sizeof(lastResponse_), lastNegativeResponse_);
    }

    static void setLastDid(uint16_t did, const char* value) {
        char didText[12];
        formatDid(did, didText, sizeof(didText));
        std::snprintf(lastDid_, sizeof(lastDid_), "%s=%s", didText, value != nullptr ? value : "--");
    }

    static void setLastDtcSummary(const char* value) {
        copyText(lastDtcSummary_, sizeof(lastDtcSummary_), value != nullptr && value[0] != '\0' ? value : "--");
    }

    static uint32_t requestCount() { return requestCount_; }
    static uint32_t sendFailureCount() { return sendFailureCount_; }
    static uint32_t timeoutCount() { return timeoutCount_; }
    static uint32_t positiveResponseCount() { return positiveResponseCount_; }
    static uint32_t negativeResponseCount() { return negativeResponseCount_; }
    static bool available() { return available_; }
    static const char* lastRequest() { return lastRequest_; }
    static const char* lastResponse() { return lastResponse_; }
    static const char* lastNegativeResponse() { return lastNegativeResponse_; }
    static const char* lastDid() { return lastDid_; }
    static const char* lastDtcSummary() { return lastDtcSummary_; }

private:
    static void copyText(char* target, std::size_t targetSize, const char* source) {
        if (target == nullptr || targetSize == 0) return;
        if (source == nullptr) source = "";
        std::snprintf(target, targetSize, "%s", source);
    }

    static inline uint32_t requestCount_ = 0;
    static inline uint32_t sendFailureCount_ = 0;
    static inline uint32_t timeoutCount_ = 0;
    static inline uint32_t positiveResponseCount_ = 0;
    static inline uint32_t negativeResponseCount_ = 0;
    static inline bool available_ = false;
    static inline char lastRequest_[128] = "--";
    static inline char lastResponse_[128] = "--";
    static inline char lastNegativeResponse_[96] = "--";
    static inline char lastDid_[80] = "--";
    static inline char lastDtcSummary_[96] = "--";
};

} // namespace Uds
