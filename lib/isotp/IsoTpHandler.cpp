#include "IsoTpHandler.h"

#if defined(ARDUINO)
#include <Arduino.h>
#include "Logger.h"
#include "SenderConfig.h"
#endif

namespace IsoTp {

bool IsoTpHandler::sendRequest(uint8_t mode, uint8_t pid) {
    const uint8_t payload[] = {mode, pid};
    return sendRequest(mode, payload, sizeof(payload));
}

bool IsoTpHandler::sendRequest(uint8_t, const uint8_t* payload, std::size_t payloadLength) {
#if defined(ARDUINO)
    if (payload == nullptr || payloadLength == 0 || payloadLength > 7) return false;

    twai_message_t request = {};
    request.identifier = FunctionalRequestId;
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
    return twai_transmit(&request, pdMS_TO_TICKS(SenderConfig::ObdTxTimeoutMs)) == ESP_OK;
#else
    (void)payload;
    (void)payloadLength;
    return false;
#endif
}

bool IsoTpHandler::receiveResponse(uint8_t expectedMode, uint8_t expectedPid, Payload& out, uint32_t timeoutMs) {
#if defined(ARDUINO)
    reassembler_.reset();
    const uint32_t start = millis();

    while (millis() - start < timeoutMs) {
        twai_message_t message = {};
        if (twai_receive(&message, pdMS_TO_TICKS(10)) != ESP_OK) continue;
        if (message.identifier < FirstResponseId || message.identifier > LastResponseId) continue;

        CanFrame frame{};
        frame.id = message.identifier;
        frame.dlc = message.data_length_code;
        memcpy(frame.data, message.data, sizeof(frame.data));

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
            Logger::warn("[ISOTP] Reassembly error");
            return false;
        }

        out = reassembler_.payload();
        if (out.length >= 2 && out.bytes[0] == 0x7F) {
            Logger::warn("[ISOTP] Negative response received");
            return false;
        }
        if (out.length >= 1 && out.bytes[0] != static_cast<uint8_t>(expectedMode | 0x40U)) {
            continue;
        }
        if (expectedPid != 0xFF && out.length >= 2 && out.bytes[1] != expectedPid) {
            continue;
        }
        Logger::debug("[ISOTP] Reassembly complete");
        return true;
    }
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
