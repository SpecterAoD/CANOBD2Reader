#pragma once

#include <cstddef>
#include <cstdint>
#include "CanRouterHub.h"
#include "IsoTpReassembler.h"

#if defined(ARDUINO)
#include <driver/twai.h>
#endif

namespace IsoTp {

class IsoTpHandler : public CanRouting::CanFrameListener {
public:
    bool sendRequest(uint8_t mode, uint8_t pid);
    bool sendRequest(uint8_t mode, const uint8_t* payload, std::size_t payloadLength);
    bool receiveResponse(uint8_t expectedMode, uint8_t expectedPid, Payload& out, uint32_t timeoutMs);
    void setRequestId(uint32_t requestId) { requestId_ = requestId; }
    uint32_t requestId() const { return requestId_; }
    bool lastResponseWasNegative() const { return lastResponseWasNegative_; }
    uint8_t lastNegativeService() const { return lastNegativeService_; }
    uint8_t lastNegativeCode() const { return lastNegativeCode_; }
    uint32_t lastResponseId() const { return lastResponseId_; }
    Status lastStatus() const { return lastStatus_; }
    void onCanFrame(const CanRouting::CanFrame& frame) override;

private:
    static constexpr std::size_t RoutedFrameQueueSize = 12;

    bool processSingleFrame(const CanFrame& frame);
    bool processFirstFrame(const CanFrame& frame);
    bool processConsecutiveFrame(const CanFrame& frame);
    bool sendFlowControl(uint32_t responseId, FlowStatus status, uint8_t blockSize, uint8_t stMinMs);
    bool popRoutedFrame(CanFrame& frame);
    void clearRoutedFrames();

    IsoTpReassembler reassembler_{};
    CanFrame routedFrames_[RoutedFrameQueueSize]{};
    std::size_t routedReadIndex_ = 0;
    std::size_t routedWriteIndex_ = 0;
    std::size_t routedCount_ = 0;
    uint32_t requestId_ = FunctionalRequestId;
    uint32_t lastResponseId_ = 0;
    Status lastStatus_ = Status::Idle;
    uint8_t lastNegativeService_ = 0;
    uint8_t lastNegativeCode_ = 0;
    bool lastResponseWasNegative_ = false;
};

}
