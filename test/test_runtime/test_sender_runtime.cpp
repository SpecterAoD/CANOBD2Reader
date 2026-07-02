#include <unity.h>
#include "SenderLoopState.h"
#include "SenderRuntimeState.h"

void setUp() {
    Runtime::SenderRuntimeState::reset();
}

void tearDown() {}

void test_consumption_requires_speed_and_fuel_rate() {
    TEST_ASSERT_FALSE(Runtime::SenderRuntimeState::hasConsumptionInput());
    Runtime::SenderRuntimeState::updateSpeed(50.0f);
    TEST_ASSERT_FALSE(Runtime::SenderRuntimeState::hasConsumptionInput());
    Runtime::SenderRuntimeState::updateFuelRate(5.0f);
    TEST_ASSERT_TRUE(Runtime::SenderRuntimeState::hasConsumptionInput());
}

void test_consumption_average_after_ten_samples() {
    Runtime::SenderRuntimeState::updateSpeed(100.0f);
    Runtime::SenderRuntimeState::updateFuelRate(7.5f);

    float average = 0.0f;
    for (int i = 0; i < 9; ++i) {
        TEST_ASSERT_FALSE(Runtime::SenderRuntimeState::addConsumptionSample(average));
    }

    TEST_ASSERT_TRUE(Runtime::SenderRuntimeState::addConsumptionSample(average));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.5f, average);
    TEST_ASSERT_EQUAL_UINT32(0, Runtime::SenderRuntimeState::consumptionSampleCount());
}

void test_sender_loop_state_tracks_can_activity() {
    Runtime::SenderLoopState state;
    state.resetForBoot(1000, 250);

    TEST_ASSERT_FALSE(state.canBusActive);
    TEST_ASSERT_FALSE(state.canRecent(1000, 200));

    state.markCanTraffic(1250);
    TEST_ASSERT_TRUE(state.canBusActive);
    TEST_ASSERT_TRUE(state.canRecent(1300, 100));
    TEST_ASSERT_FALSE(state.canRecent(1400, 100));
}

void test_sender_loop_state_resets_timestamps() {
    Runtime::SenderLoopState state;
    state.markCanTraffic(500);
    state.heartbeatCount = 3;
    state.canDriverReady = true;
    state.espNowReady = true;

    state.resetForBoot(50, 200);

    TEST_ASSERT_EQUAL_UINT32(0, state.lastCanMessageAt);
    TEST_ASSERT_EQUAL_UINT32(0, state.lastHeartbeatSentAt);
    TEST_ASSERT_EQUAL_UINT32(0, state.lastTwaiStatusLogAt);
    TEST_ASSERT_EQUAL_UINT32(0, state.heartbeatCount);
    TEST_ASSERT_FALSE(state.canBusActive);
    TEST_ASSERT_FALSE(state.canDriverReady);
    TEST_ASSERT_FALSE(state.espNowReady);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_consumption_requires_speed_and_fuel_rate);
    RUN_TEST(test_consumption_average_after_ten_samples);
    RUN_TEST(test_sender_loop_state_tracks_can_activity);
    RUN_TEST(test_sender_loop_state_resets_timestamps);
    return UNITY_END();
}
