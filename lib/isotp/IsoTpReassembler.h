#pragma once

#include "IsoTpTypes.h"

namespace IsoTp {

class IsoTpReassembler {
public:
    void reset();
    Status processFrame(const CanFrame& frame);
    const Payload& payload() const { return payload_; }
    uint8_t expectedSequenceNumber() const { return expectedSequenceNumber_; }

private:
    Status processSingleFrame(const CanFrame& frame);
    Status processFirstFrame(const CanFrame& frame);
    Status processConsecutiveFrame(const CanFrame& frame);

    Payload payload_{};
    std::size_t expectedLength_ = 0;
    uint8_t expectedSequenceNumber_ = 1;
    bool receiving_ = false;
    uint32_t activeResponseId_ = 0;
};

}
