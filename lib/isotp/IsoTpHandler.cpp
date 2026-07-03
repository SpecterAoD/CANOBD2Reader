#include "IsoTpHandler.h"

#if defined(ARDUINO)
#include <Arduino.h>
#include "Logger.h"
#include "config/SenderConfig.h"
#endif

namespace IsoTp {

bool IsoTpHandler::sendRequest(uint8_t mode, uint8_t pid) {
    const uint8_t payload[] = {mode, pid};
    return sendRequest(mode, payload, sizeof(payload));
}

bool IsoTpHandler::sendRequest(uint8_t mode, const uint8_t* payload, std::size_t payloadLength) {
#if defined(ARDUINO)
    if (payload == nullptr || payloadLength == 0 || payloadLength > 7) {
        Logger::warn("[ISOTP] Invalid request payload");
        return false;
    }

    twai_message_t request = {};
    request.identifier = requestId_;
    request.extd = 0;
    request.rtr = 0;
    request.data_length_code = 8;
    request.data[0] = static_cast<uint8_t>(payloadLength);
    for (std::size_t index = 0; index < payloadLength; ++index) {
        request.data[index + 1] = payload[index];
    }
    for (std::size_t index = payloadLength + 1; index < 8; ++index) {
        request.data[index] = 0x55;
    }
    const esp_err_t result = twai_transmit(&request, pdMS_TO_TICKS(SenderConfig::ObdTxTimeoutMs));
    if (result == ESP_OK) {
        char line[96];
        snprintf(line, sizeof(line), "[ISOTP] Request sent id=0x%03lX mode=0x%02X pid=0x%02X len=%u",
                 static_cast<unsigned long>(request.identifier),
                 mode,
                 payloadLength > 1 ? payload[1] : 0xFF,
                 static_cast<unsigned>(payloadLength));
        Logger::obd(line);
        return true;
    }

    char line[96];
    snprintf(line, sizeof(line), "[ISOTP] Request transmit failed err=%d mode=0x%02X pid=0x%02X",
             static_cast<int>(result),
             mode,
             payloadLength > 1 ? payload[1] : 0xFF);
    Logger::warn(line);
    return false;
#else
    (void)mode;
    (void)payload;
    (void)payloadLength;
    return false;
#endif
}

bool IsoTpHandler::receiveResponse(uint8_t expectedMode, uint8_t expectedPid, Payload& out, uint32_t timeoutMs) {
#if defined(ARDUINO)
    reassembler_.reset();
    clearRoutedFrames();
    lastStatus_ = Status::Idle;
    lastResponseId_ = 0;
    lastNegativeService_ = 0;
    lastNegativeCode_ = 0;
    lastResponseWasNegative_ = false;
    const uint32_t start = millis();
    const bool registered = CanRouting::registerListener(*this);
    if (!registered) {
        Logger::warn("[ISOTP] Failed to register CAN router listener");
        lastStatus_ = Status::Aborted;
        return false;
    }

    while (millis() - start < timeoutMs) {
        CanRouting::pumpFrames();

        CanFrame frame{};
        if (!popRoutedFrame(frame)) {
            delay(2);
            continue;
        }

        const auto status = reassembler_.processFrame(frame);
        if (status == Status::InProgress) {
            if (((frame.data[0] >> 4U) & 0x0FU) == static_cast<uint8_t>(FrameType::First)) {
                Logger::debug("[ISOTP] First Frame received");
                sendFlowControl(frame.id, FlowStatus::ContinueToSend,
                                SenderConfig::IsoTpBlockSize,
                                SenderConfig::IsoTpStMinMs);
                Logger::debug("[ISOTP] Flow Control sent");
            } else {
                Logger::debug("[ISOTP] Consecutive Frame received");
            }
            continue;
        }
        if (status != Status::Complete) {
            lastStatus_ = status;
            Logger::warn("[ISOTP] Reassembly error");
            CanRouting::unregisterListener(*this);
            return false;
        }

        out = reassembler_.payload();
        lastStatus_ = Status::Complete;
        lastResponseId_ = out.responseId;
        char responseLine[112];
        snprintf(responseLine, sizeof(responseLine),
                 "[ISOTP] Response id=0x%03lX len=%u first=0x%02X",
                 static_cast<unsigned long>(out.responseId),
                 static_cast<unsigned>(out.length),
                 out.length > 0 ? out.bytes[0] : 0);
        Logger::obd(responseLine);
        if (out.length >= 2 && out.bytes[0] == 0x7F) {
            lastResponseWasNegative_ = true;
            lastNegativeService_ = out.length > 1 ? out.bytes[1] : 0;
            lastNegativeCode_ = out.length > 2 ? out.bytes[2] : 0;
            char negativeLine[96];
            snprintf(negativeLine, sizeof(negativeLine),
                     "[ISOTP] Negative response service=0x%02X nrc=0x%02X",
                     lastNegativeService_,
                     lastNegativeCode_);
            Logger::warn(negativeLine);
            CanRouting::unregisterListener(*this);
            return false;
        }
        if (out.length >= 1 && out.bytes[0] != static_cast<uint8_t>(expectedMode | 0x40U)) {
            continue;
        }
        if (expectedPid != 0xFF && out.length >= 2 && out.bytes[1] != expectedPid) {
            continue;
        }
        Logger::debug("[ISOTP] Reassembly complete");
        CanRouting::unregisterListener(*this);
        return true;
    }
    CanRouting::unregisterListener(*this);
    lastStatus_ = Status::Timeout;
    Logger::warn("[ISOTP] Timeout waiting for response");
    return false;
#else
    (void)expectedMode;
    (void)expectedPid;
    (void)out;
    (void)timeoutMs;
    return false;
#endif
}

void IsoTpHandler::onCanFrame(const CanRouting::CanFrame& frame) {
    if (frame.id < FirstResponseId || frame.id > LastResponseId) return;
    if (frame.length == 0 || frame.length > 8) return;
    if (routedCount_ >= RoutedFrameQueueSize) {
        lastStatus_ = Status::BufferOverflow;
        return;
    }

    CanFrame& target = routedFrames_[routedWriteIndex_];
    target = CanFrame{};
    target.id = frame.id;
    target.dlc = frame.length;
    for (uint8_t index = 0; index < target.dlc; ++index) {
        target.data[index] = frame.data[index];
    }
    routedWriteIndex_ = (routedWriteIndex_ + 1U) % RoutedFrameQueueSize;
    ++routedCount_;
}

bool IsoTpHandler::popRoutedFrame(CanFrame& frame) {
    if (routedCount_ == 0) return false;
    frame = routedFrames_[routedReadIndex_];
    routedReadIndex_ = (routedReadIndex_ + 1U) % RoutedFrameQueueSize;
    --routedCount_;
    return true;
}

void IsoTpHandler::clearRoutedFrames() {
    routedReadIndex_ = 0;
    routedWriteIndex_ = 0;
    routedCount_ = 0;
    for (auto& frame : routedFrames_) frame = CanFrame{};
}

bool IsoTpHandler::processSingleFrame(const CanFrame& frame) {
    return reassembler_.processFrame(frame) == Status::Complete;
}

bool IsoTpHandler::processFirstFrame(const CanFrame& frame) {
    return reassembler_.processFrame(frame) == Status::InProgress;
}

bool IsoTpHandler::processConsecutiveFrame(const CanFrame& frame) {
    const auto status = reassembler_.processFrame(frame);
    return status == Status::InProgress || status == Status::Complete;
}

bool IsoTpHandler::sendFlowControl(uint32_t responseId, FlowStatus status, uint8_t blockSize, uint8_t stMinMs) {
#if defined(ARDUINO)
    twai_message_t fc = {};
    fc.identifier = responseId - 8U;
    fc.extd = 0;
    fc.rtr = 0;
    fc.data_length_code = 8;
    fc.data[0] = static_cast<uint8_t>(0x30U | (static_cast<uint8_t>(status) & 0x0FU));
    fc.data[1] = blockSize;
    fc.data[2] = stMinMs;
    for (uint8_t index = 3; index < 8; ++index) fc.data[index] = 0x55;
    return twai_transmit(&fc, pdMS_TO_TICKS(SenderConfig::ObdTxTimeoutMs)) == ESP_OK;
#else
    (void)responseId;
    (void)status;
    (void)blockSize;
    (void)stMinMs;
    return false;
#endif
}

}
