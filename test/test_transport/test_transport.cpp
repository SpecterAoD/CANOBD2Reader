#include <unity.h>
#include "EspNowTelemetryTransport.h"
#include "TelemetryCodec.h"

void setUp() {}
void tearDown() {}

void test_native_transport_accepts_encoded_packet() {
    Telemetry::TelemetryPacket packet{};
    Telemetry::TelemetryCodec::encodeText(packet,
                                          Telemetry::PacketType::Status,
                                          42,
                                          1000,
                                          "STATUS,HEARTBEAT,Heartbeat,1,,OK,42");
    TEST_ASSERT_EQUAL(ESP_OK, Transport::sendTelemetryPacket(packet));
}

void test_transport_log_result_is_native_safe() {
    Transport::logTelemetrySendResult(ESP_OK, 42, "STATUS,FW,Firmware,V1,,OK,42");
    TEST_PASS();
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_native_transport_accepts_encoded_packet);
    RUN_TEST(test_transport_log_result_is_native_safe);
    return UNITY_END();
}
