#pragma once

#include <cstddef>
#include <cstdint>
#include "TelemetryPacket.h"

namespace Telemetry {

class TelemetryCodec {
public:
    static bool encode(TelemetryPacket& packet,
                       PacketType type,
                       uint32_t sequence,
                       uint32_t timestamp,
                       const uint8_t* payload,
                       std::size_t payloadLength);

    static bool encodeText(TelemetryPacket& packet,
                           PacketType type,
                           uint32_t sequence,
                           uint32_t timestamp,
                           const char* payload);

    static DecodeStatus decode(const uint8_t* bytes,
                               std::size_t length,
                               TelemetryPacket& packet);

    static uint16_t calculateCrc(const TelemetryPacket& packet);
};

}
