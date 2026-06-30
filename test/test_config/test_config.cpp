#include <unity.h>
#include "ProjectConfig.h"
#include "SenderConfig.h"
#include "DisplayConfig.h"

void setUp() {}
void tearDown() {}

void test_project_protocol_constants() {
    TEST_ASSERT_EQUAL_HEX16(0xCA02, ProjectConfig::ProtocolMagic);
    TEST_ASSERT_TRUE(ProjectConfig::ProtocolVersion > 0);
    TEST_ASSERT_EQUAL_UINT8(1, ProjectConfig::EspNowChannel);
}

void test_buffer_limits() {
    TEST_ASSERT_TRUE(ProjectConfig::TelemetryPayloadSize <= 200);
    TEST_ASSERT_EQUAL_UINT(4095, ProjectConfig::MaxIsoTpPayload);
}

void test_sender_timing() {
    TEST_ASSERT_TRUE(SenderConfig::ObdPollIntervalMs >= 50);
    TEST_ASSERT_TRUE(SenderConfig::ObdResponseTimeoutMs >= SenderConfig::ObdTxTimeoutMs);
}

void test_display_queue() {
    TEST_ASSERT_TRUE(DisplayConfigValues::TelemetryQueueLength >= 4);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_project_protocol_constants);
    RUN_TEST(test_buffer_limits);
    RUN_TEST(test_sender_timing);
    RUN_TEST(test_display_queue);
    return UNITY_END();
}
