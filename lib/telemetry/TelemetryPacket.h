#pragma once

#include <cstddef>
#include <cstdint>
#include "config/ProjectConfig.h"

namespace Telemetry {

enum class PacketType : uint8_t {
    Text = 1,
    Status = 2,
    Obd = 3,
    Can = 4,
    Diagnostic = 5
};

struct TelemetryPacket {
    uint16_t magic = ProjectConfig::ProtocolMagic;
    uint8_t version = ProjectConfig::ProtocolVersion;
    uint8_t type = static_cast<uint8_t>(PacketType::Text);
    uint32_t sequence = 0;
    uint32_t timestamp = 0;
    uint16_t payloadLength = 0;
    uint8_t payload[ProjectConfig::TelemetryPayloadSize]{};
    uint16_t crc = 0;
};

static_assert(sizeof(TelemetryPacket) <= 250, "ESP-NOW packet must fit into one vendor action frame");

enum class DecodeStatus : uint8_t {
    Ok,
    InvalidLength,
    InvalidMagic,
    InvalidVersion,
    PayloadTooLarge,
    CrcMismatch
};

}
