// Log throttling and ESP-NOW backoff.
#include <unity.h>

#include "EventThrottle.h"
#include "LinkBackoff.h"

void setUp() {}
void tearDown() {}

void test_throttle_reports_first_event_then_summaries() {
    Logging::EventThrottle throttle(5000);
    TEST_ASSERT_TRUE(throttle.record(100));
    TEST_ASSERT_EQUAL_UINT32(1, throttle.count());
    for (uint32_t t = 200; t < 5100; t += 100) TEST_ASSERT_FALSE(throttle.record(t));
    TEST_ASSERT_TRUE(throttle.record(5100));
    TEST_ASSERT_EQUAL_UINT32(50, throttle.count());  // 49 suppressed + this one
    TEST_ASSERT_EQUAL_UINT32(0, throttle.suppressed());
}

void test_backoff_only_after_consecutive_failures() {
    Transport::LinkBackoff link(5, 1000);
    for (int i = 0; i < 4; ++i) {
        TEST_ASSERT_TRUE(link.allowSend(i * 10));
        link.reportResult(false);
    }
    link.reportResult(true);  // a success in between resets the streak
    TEST_ASSERT_FALSE(link.offline());
}

void test_backoff_limits_to_one_probe_per_interval() {
    Transport::LinkBackoff link(5, 1000);
    for (int i = 0; i < 5; ++i) link.reportResult(false);
    TEST_ASSERT_TRUE(link.offline());
    int sent = 0;
    for (uint32_t t = 0; t < 3000; t += 100) {
        if (link.allowSend(t)) {
            ++sent;
            link.reportResult(false);
        }
    }
    TEST_ASSERT_EQUAL(3, sent);  // t = 0, 1000, 2000
    TEST_ASSERT_EQUAL_UINT32(27, link.skipped());
}

void test_backoff_ends_on_first_success() {
    Transport::LinkBackoff link(5, 1000);
    for (int i = 0; i < 5; ++i) link.reportResult(false);
    TEST_ASSERT_TRUE(link.allowSend(0));
    link.reportResult(true);
    TEST_ASSERT_FALSE(link.offline());
    TEST_ASSERT_TRUE(link.allowSend(10));
    TEST_ASSERT_TRUE(link.allowSend(20));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_throttle_reports_first_event_then_summaries);
    RUN_TEST(test_backoff_only_after_consecutive_failures);
    RUN_TEST(test_backoff_limits_to_one_probe_per_interval);
    RUN_TEST(test_backoff_ends_on_first_success);
    return UNITY_END();
}
