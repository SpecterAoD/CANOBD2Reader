#include <unity.h>
#include "BoostCalculator.h"

void setUp() {}
void tearDown() {}

void test_boost_with_real_barometric_pressure() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.80f, Obd::calculateBoostPressureBar(180.0f, 100.0f));
}

void test_boost_with_fallback_barometric_pressure() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.787f,
                             Obd::calculateBoostPressureBarWithFallback(180.0f, false, 0.0f));
}

void test_boost_keeps_negative_vacuum_value() {
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.20f, Obd::calculateBoostPressureBar(80.0f, 100.0f));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_boost_with_real_barometric_pressure);
    RUN_TEST(test_boost_with_fallback_barometric_pressure);
    RUN_TEST(test_boost_keeps_negative_vacuum_value);
    return UNITY_END();
}
