#include "TelemetryCodec.h"
#include <cstring>
#include "Crc16.h"

namespace Telemetry {

namespace {
constexpr std::size_t crcOffset() {
    return offsetof(TelemetryPacket, crc);
}
}

bool TelemetryCodec::encode(TelemetryPacket& packet,
                            PacketType type,
                            uint32_t sequence,
                            uint32_t timestamp,
                            const uint8_t* payload,
                            std::size_t payloadLength) {
    if (payloadLength > ProjectConfig::TelemetryPayloadSize) return false;
    if (payloadLength > 0 && payload == nullptr) return false;

    packet = TelemetryPacket{};
    packet.magic = ProjectConfig::ProtocolMagic;
    packet.version = ProjectConfig::ProtocolVersion;
    packet.type = static_cast<uint8_t>(type);
    packet.sequence = sequence;
    packet.timestamp = timestamp;
    packet.payloadLength = static_cast<uint16_t>(payloadLength);
    if (payloadLength > 0) {
        std::memcpy(packet.payload, payload, payloadLength);
    }
    packet.crc = calculateCrc(packet);
    return true;
}

bool TelemetryCodec::encodeText(TelemetryPacket& packet,
                                PacketType type,
                                uint32_t sequence,
                                uint32_t timestamp,
                                const char* payload) {
    if (payload == nullptr) payload = "";
    const std::size_t length = strnlen(payload, ProjectConfig::TelemetryPayloadSize);
    return encode(packet, type, sequence, timestamp,
                  reinterpret_cast<const uint8_t*>(payload), length);
}

DecodeStatus TelemetryCodec::decode(const uint8_t* bytes,
                                    std::size_t length,
                                    TelemetryPacket& packet) {
    if (bytes == nullptr || length != sizeof(TelemetryPacket)) return DecodeStatus::InvalidLength;

    std::memcpy(&packet, bytes, sizeof(TelemetryPacket));
    if (packet.magic != ProjectConfig::ProtocolMagic) return DecodeStatus::InvalidMagic;
    if (packet.version != ProjectConfig::ProtocolVersion) return DecodeStatus::InvalidVersion;
    if (packet.payloadLength > ProjectConfig::TelemetryPayloadSize) return DecodeStatus::PayloadTooLarge;

    const uint16_t expected = calculateCrc(packet);
    if (expected != packet.crc) return DecodeStatus::CrcMismatch;
    return DecodeStatus::Ok;
}

uint16_t TelemetryCodec::calculateCrc(const TelemetryPacket& packet) {
    return Crc16::modbus(reinterpret_cast<const uint8_t*>(&packet), crcOffset());
}

}
