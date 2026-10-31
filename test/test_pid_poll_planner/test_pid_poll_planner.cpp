// Fair OBD polling: fast PIDs stay fast, slow PIDs are never starved.
#include <unity.h>

#include <map>

#include "PidPollPlanner.h"

void setUp() {}
void tearDown() {}

namespace {
// Simulates a bus where every request takes `requestMs` and counts polls.
std::map<uint8_t, int> run(Obd::PidPollPlanner& planner, uint32_t durationMs, uint32_t requestMs) {
    std::map<uint8_t, int> polls;
    uint32_t now = 0;
    while (now < durationMs) {
        uint8_t pid = 0;
        if (planner.next(now, pid)) {
            ++polls[pid];
            now += requestMs;
            planner.markPolled(pid, now);
        } else {
            now += 10;
        }
    }
    return polls;
}
} // namespace

void test_slow_pids_are_not_starved_by_fast_pids() {
    Obd::PidPollPlanner planner;
    const uint8_t fast[] = {0x0C, 0x0D, 0x05, 0x04, 0x42};
    const uint8_t slow[] = {0x0B, 0x0F, 0x33, 0x10, 0x11, 0x2F, 0x1F, 0x46, 0x5C, 0x5E};
    for (uint8_t pid : fast) planner.add(pid, 200);
    for (uint8_t pid : slow) planner.add(pid, 2000);

    // Busy bus: 35 ms per request means the fast set alone needs 175 ms of 200.
    const auto polls = run(planner, 60000, 35);
    for (uint8_t pid : slow) {
        // 60 s / 2 s = 30 slots; allow scheduling jitter.
        TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(20, polls.at(pid), "slow PID starved");
    }
    for (uint8_t pid : fast) {
        TEST_ASSERT_GREATER_OR_EQUAL_MESSAGE(150, polls.at(pid), "fast PID too slow");
    }
}

void test_never_polled_pids_go_first_in_order() {
    Obd::PidPollPlanner planner;
    planner.add(0x0C, 200);
    planner.add(0x5C, 2000);
    uint8_t pid = 0;
    TEST_ASSERT_TRUE(planner.next(0, pid));
    TEST_ASSERT_EQUAL_HEX8(0x0C, pid);
    planner.markPolled(0x0C, 0);
    TEST_ASSERT_TRUE(planner.next(0, pid));
    TEST_ASSERT_EQUAL_HEX8(0x5C, pid);
}

void test_nothing_due_returns_false() {
    Obd::PidPollPlanner planner;
    planner.add(0x0C, 200);
    planner.markPolled(0x0C, 1000);
    uint8_t pid = 0;
    TEST_ASSERT_FALSE(planner.next(1100, pid));
    TEST_ASSERT_TRUE(planner.next(1200, pid));
}

void test_unsupported_pids_are_skipped() {
    Obd::PidPollPlanner planner;
    planner.add(0x0C, 200);
    planner.add(0x5C, 200);
    planner.setSupported(0x0C, false);
    uint8_t pid = 0;
    TEST_ASSERT_TRUE(planner.next(0, pid));
    TEST_ASSERT_EQUAL_HEX8(0x5C, pid);
}

void test_duplicates_and_zero_interval_are_rejected() {
    Obd::PidPollPlanner planner;
    TEST_ASSERT_TRUE(planner.add(0x0C, 200));
    TEST_ASSERT_FALSE(planner.add(0x0C, 300));
    TEST_ASSERT_FALSE(planner.add(0x0D, 0));
    TEST_ASSERT_EQUAL(1, planner.size());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_slow_pids_are_not_starved_by_fast_pids);
    RUN_TEST(test_never_polled_pids_go_first_in_order);
    RUN_TEST(test_nothing_due_returns_false);
    RUN_TEST(test_unsupported_pids_are_skipped);
    RUN_TEST(test_duplicates_and_zero_interval_are_rejected);
    return UNITY_END();
}
