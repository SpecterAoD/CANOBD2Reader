#pragma once

#include <cstddef>
#include <cstdint>
#include "IsoTpReassembler.h"

#if defined(ARDUINO)
#include <driver/twai.h>
#endif

namespace IsoTp {

class IsoTpHandler {
public:
    bool sendRequest(uint8_t mode, uint8_t pid);
    bool sendRequest(uint8_t mode, const uint8_t* payload, std::size_t payloadLength);
    bool receiveResponse(uint8_t expectedMode, uint8_t expectedPid, Payload& out, uint32_t timeoutMs);

private:
    bool processSingleFrame(const CanFrame& frame);
    bool processFirstFrame(const CanFrame& frame);
    bool processConsecutiveFrame(const CanFrame& frame);
    bool sendFlowControl(uint32_t responseId, FlowStatus status, uint8_t blockSize, uint8_t stMinMs);

    IsoTpReassembler reassembler_{};
};

}
