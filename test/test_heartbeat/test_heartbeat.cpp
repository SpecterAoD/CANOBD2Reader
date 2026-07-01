#include <unity.h>
#include "SenderConfig.h"
#include "StatusLogic.h"

void setUp() {}
void tearDown() {}

void test_heartbeat_is_due_without_previous_send() {
    TEST_ASSERT_TRUE(StatusLogic::isHeartbeatDue(100, 0, SenderConfig::HeartbeatIntervalMs));
}

void test_heartbeat_interval_controls_send_rate() {
    TEST_ASSERT_FALSE(StatusLogic::isHeartbeatDue(1500, 1000, SenderConfig::HeartbeatIntervalMs));
    TEST_ASSERT_TRUE(StatusLogic::isHeartbeatDue(2000, 1000, SenderConfig::HeartbeatIntervalMs));
}

void test_sequence_gap_counts_packet_loss() {
    TEST_ASSERT_EQUAL_UINT32(0, StatusLogic::packetLossFromSequence(0, 7));
    TEST_ASSERT_EQUAL_UINT32(0, StatusLogic::packetLossFromSequence(7, 8));
    TEST_ASSERT_EQUAL_UINT32(2, StatusLogic::packetLossFromSequence(7, 10));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_heartbeat_is_due_without_previous_send);
    RUN_TEST(test_heartbeat_interval_controls_send_rate);
    RUN_TEST(test_sequence_gap_counts_packet_loss);
    return UNITY_END();
}
