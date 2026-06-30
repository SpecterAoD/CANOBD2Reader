#include <unity.h>
#include <cstring>
#include "TelemetryCodec.h"

void setUp() {}
void tearDown() {}

void test_telemetry_encode_decode() {
    Telemetry::TelemetryPacket encoded{};
    TEST_ASSERT_TRUE(Telemetry::TelemetryCodec::encodeText(encoded, Telemetry::PacketType::Obd, 7, 1234, "OBD,0C,RPM,1000,rpm,OK,7"));

    Telemetry::TelemetryPacket decoded{};
    TEST_ASSERT_EQUAL(static_cast<int>(Telemetry::DecodeStatus::Ok),
                      static_cast<int>(Telemetry::TelemetryCodec::decode(reinterpret_cast<const uint8_t*>(&encoded), sizeof(encoded), decoded)));
    TEST_ASSERT_EQUAL_UINT16(ProjectConfig::ProtocolMagic, decoded.magic);
    TEST_ASSERT_EQUAL_UINT8(ProjectConfig::ProtocolVersion, decoded.version);
    TEST_ASSERT_EQUAL_UINT32(7, decoded.sequence);
}

void test_telemetry_rejects_magic() {
    Telemetry::TelemetryPacket encoded{};
    Telemetry::TelemetryCodec::encodeText(encoded, Telemetry::PacketType::Text, 1, 1, "x");
    encoded.magic = 0x1234;
    encoded.crc = Telemetry::TelemetryCodec::calculateCrc(encoded);
    Telemetry::TelemetryPacket decoded{};
    TEST_ASSERT_EQUAL(static_cast<int>(Telemetry::DecodeStatus::InvalidMagic),
                      static_cast<int>(Telemetry::TelemetryCodec::decode(reinterpret_cast<const uint8_t*>(&encoded), sizeof(encoded), decoded)));
}

void test_telemetry_rejects_crc() {
    Telemetry::TelemetryPacket encoded{};
    Telemetry::TelemetryCodec::encodeText(encoded, Telemetry::PacketType::Text, 1, 1, "x");
    encoded.payload[0] = 'y';
    Telemetry::TelemetryPacket decoded{};
    TEST_ASSERT_EQUAL(static_cast<int>(Telemetry::DecodeStatus::CrcMismatch),
                      static_cast<int>(Telemetry::TelemetryCodec::decode(reinterpret_cast<const uint8_t*>(&encoded), sizeof(encoded), decoded)));
}

void test_telemetry_rejects_length() {
    Telemetry::TelemetryPacket decoded{};
    uint8_t bytes[4] = {};
    TEST_ASSERT_EQUAL(static_cast<int>(Telemetry::DecodeStatus::InvalidLength),
                      static_cast<int>(Telemetry::TelemetryCodec::decode(bytes, sizeof(bytes), decoded)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_telemetry_encode_decode);
    RUN_TEST(test_telemetry_rejects_magic);
    RUN_TEST(test_telemetry_rejects_crc);
    RUN_TEST(test_telemetry_rejects_length);
    return UNITY_END();
}
