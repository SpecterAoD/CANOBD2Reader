#include <unity.h>
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

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_consumption_requires_speed_and_fuel_rate);
    RUN_TEST(test_consumption_average_after_ten_samples);
    return UNITY_END();
}
