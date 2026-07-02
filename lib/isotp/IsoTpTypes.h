#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include "config/ProjectConfig.h"

namespace IsoTp {

constexpr std::size_t MaxPayload = ProjectConfig::MaxIsoTpPayload;
constexpr uint32_t FunctionalRequestId = 0x7DF;
constexpr uint32_t PhysicalRequestId = 0x7E0;
constexpr uint32_t FirstResponseId = 0x7E8;
constexpr uint32_t LastResponseId = 0x7EF;

enum class FrameType : uint8_t {
    Single = 0x0,
    First = 0x1,
    Consecutive = 0x2,
    FlowControl = 0x3
};

enum class FlowStatus : uint8_t {
    ContinueToSend = 0x0,
    Wait = 0x1,
    Overflow = 0x2
};

enum class Status : uint8_t {
    Idle,
    InProgress,
    Complete,
    Timeout,
    SequenceError,
    BufferOverflow,
    InvalidLength,
    InvalidFrameType,
    DuplicateFrame,
    FlowOverflow,
    Aborted
};

struct CanFrame {
    uint32_t id = 0;
    uint8_t dlc = 0;
    uint8_t data[8]{};
};

struct Payload {
    std::array<uint8_t, MaxPayload> bytes{};
    std::size_t length = 0;
    uint32_t responseId = 0;
};

}
