#include "IsoTpReassembler.h"
#include <cstring>

namespace IsoTp {

void IsoTpReassembler::reset() {
    payload_ = Payload{};
    expectedLength_ = 0;
    expectedSequenceNumber_ = 1;
    receiving_ = false;
    activeResponseId_ = 0;
}

Status IsoTpReassembler::processFrame(const CanFrame& frame) {
    if (frame.dlc == 0 || frame.dlc > 8) return Status::InvalidLength;
    if (frame.id < FirstResponseId || frame.id > LastResponseId) return Status::InvalidLength;

    const auto type = static_cast<FrameType>((frame.data[0] >> 4U) & 0x0FU);
    switch (type) {
        case FrameType::Single: return processSingleFrame(frame);
        case FrameType::First: return processFirstFrame(frame);
        case FrameType::Consecutive: return processConsecutiveFrame(frame);
        case FrameType::FlowControl: return Status::InvalidFrameType;
    }
    return Status::InvalidFrameType;
}

Status IsoTpReassembler::processSingleFrame(const CanFrame& frame) {
    const uint8_t length = frame.data[0] & 0x0F;
    if (length == 0 || length > 7 || frame.dlc < static_cast<uint8_t>(length + 1)) {
        reset();
        return Status::InvalidLength;
    }
    reset();
    payload_.responseId = frame.id;
    payload_.length = length;
    std::memcpy(payload_.bytes.data(), &frame.data[1], length);
    return Status::Complete;
}

Status IsoTpReassembler::processFirstFrame(const CanFrame& frame) {
    if (frame.dlc < 8) {
        reset();
        return Status::InvalidLength;
    }

    const std::size_t totalLength =
        (static_cast<std::size_t>(frame.data[0] & 0x0F) << 8U) | frame.data[1];
    if (totalLength <= 6) {
        reset();
        return Status::InvalidLength;
    }
    if (totalLength > MaxPayload) {
        reset();
        return Status::BufferOverflow;
    }

    reset();
    receiving_ = true;
    activeResponseId_ = frame.id;
    expectedLength_ = totalLength;
    payload_.responseId = frame.id;
    payload_.length = 6;
    std::memcpy(payload_.bytes.data(), &frame.data[2], 6);
    expectedSequenceNumber_ = 1;
    return payload_.length >= expectedLength_ ? Status::Complete : Status::InProgress;
}

Status IsoTpReassembler::processConsecutiveFrame(const CanFrame& frame) {
    if (!receiving_) return Status::DuplicateFrame;
    if (frame.id != activeResponseId_) return Status::InvalidLength;

    const uint8_t sequenceNumber = frame.data[0] & 0x0F;
    if (sequenceNumber != expectedSequenceNumber_) {
        reset();
        return Status::SequenceError;
    }

    const std::size_t remaining = expectedLength_ - payload_.length;
    const std::size_t copyLength = remaining < 7 ? remaining : 7;
    if (frame.dlc < static_cast<uint8_t>(copyLength + 1)) {
        reset();
        return Status::InvalidLength;
    }
    if (payload_.length + copyLength > MaxPayload) {
        reset();
        return Status::BufferOverflow;
    }

    std::memcpy(payload_.bytes.data() + payload_.length, &frame.data[1], copyLength);
    payload_.length += copyLength;
    expectedSequenceNumber_ = static_cast<uint8_t>((expectedSequenceNumber_ + 1U) & 0x0FU);

    if (payload_.length >= expectedLength_) {
        receiving_ = false;
        return Status::Complete;
    }
    return Status::InProgress;
}

}
