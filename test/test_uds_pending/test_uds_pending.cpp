#include <unity.h>

#include "UdsCapability.h"

void setUp() {}
void tearDown() {}

void test_response_pending_is_not_terminal_error() {
    TEST_ASSERT_TRUE(Capabilities::isUdsResponsePending(0x78));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Capabilities::CapabilityStatus::Pending),
                          static_cast<int>(Capabilities::statusForNegativeResponse(0x78)));
    TEST_ASSERT_FALSE(Capabilities::isTerminal(Capabilities::CapabilityStatus::Pending));
}

void test_unsupported_negative_responses_are_classified() {
    TEST_ASSERT_TRUE(Capabilities::isUdsUnsupportedNrc(0x11));
    TEST_ASSERT_TRUE(Capabilities::isUdsUnsupportedNrc(0x12));
    TEST_ASSERT_TRUE(Capabilities::isUdsUnsupportedNrc(0x31));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(Capabilities::CapabilityStatus::Unsupported),
                          static_cast<int>(Capabilities::statusForNegativeResponse(0x31)));
}

void test_pending_total_timeout_window() {
    TEST_ASSERT_TRUE(Capabilities::shouldContinueWaitingForPending(1000, 3999, 3000));
    TEST_ASSERT_FALSE(Capabilities::shouldContinueWaitingForPending(1000, 4000, 3000));
}

void test_ecu_scan_range() {
    TEST_ASSERT_TRUE(Capabilities::isUdsRequestIdInScanRange(0x7E0));
    TEST_ASSERT_TRUE(Capabilities::isUdsRequestIdInScanRange(0x7E7));
    TEST_ASSERT_FALSE(Capabilities::isUdsRequestIdInScanRange(0x7DF));
    TEST_ASSERT_EQUAL_HEX32(0x7E8, Capabilities::udsResponseIdForRequestId(0x7E0));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_response_pending_is_not_terminal_error);
    RUN_TEST(test_unsupported_negative_responses_are_classified);
    RUN_TEST(test_pending_total_timeout_window);
    RUN_TEST(test_ecu_scan_range);
    return UNITY_END();
}
