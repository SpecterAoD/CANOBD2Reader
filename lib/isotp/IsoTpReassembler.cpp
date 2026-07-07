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
        // Flow Control frames are sent by the tester, not received as ECU responses.
        // Seeing one here means the frame arrived on the wrong path; discard it.
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
    // ISO 15765-2 §6.5.3: a First Frame must fill the full 8-byte CAN payload.
    // A shorter frame is malformed and cannot carry the mandatory length field.
    if (frame.dlc < 8) {
        reset();
        return Status::InvalidLength;
    }

    // The 12-bit length field encodes the total payload byte count across all frames.
    // Anything ≤ 6 fits entirely in a Single Frame; using a First Frame for it is a
    // protocol violation and would leave the sequence counter in an undefined state.
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
    // Lock reassembly to the responding ECU's CAN ID so that frames from other
    // ECUs on the same bus cannot corrupt an in-progress multi-frame transfer.
    activeResponseId_ = frame.id;
    expectedLength_ = totalLength;
    payload_.responseId = frame.id;
    payload_.length = 6;
    std::memcpy(payload_.bytes.data(), &frame.data[2], 6);
    expectedSequenceNumber_ = 1;
    return payload_.length >= expectedLength_ ? Status::Complete : Status::InProgress;
}

Status IsoTpReassembler::processConsecutiveFrame(const CanFrame& frame) {
    // A Consecutive Frame received outside an active multi-frame transfer has
    // no valid context to attach to; reject it to avoid unintended reassembly.
    if (!receiving_) return Status::DuplicateFrame;
    // Reject frames from a different ECU to prevent cross-talk from corrupting
    // the reassembly of an in-progress response.
    if (frame.id != activeResponseId_) return Status::InvalidLength;

    const uint8_t sequenceNumber = frame.data[0] & 0x0F;
    // A gap in sequence numbers means at least one frame was dropped. Continuing
    // would silently assemble corrupt data, so abort and require a fresh request.
    if (sequenceNumber != expectedSequenceNumber_) {
        reset();
        return Status::SequenceError;
    }

    const std::size_t remaining = expectedLength_ - payload_.length;
    // The final Consecutive Frame may carry fewer than 7 payload bytes; copy only
    // what the declared length requires, ignoring any padding bytes beyond it.
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
    // The 4-bit sequence counter wraps from 15 back to 0 per ISO 15765-2 §6.5.4.
    expectedSequenceNumber_ = static_cast<uint8_t>((expectedSequenceNumber_ + 1U) & 0x0FU);

    if (payload_.length >= expectedLength_) {
        receiving_ = false;
        return Status::Complete;
    }
    return Status::InProgress;
}

}
