#include <unity.h>
#include "PidDecoder.h"

void test_pid_rpm() {
    const uint8_t data[] = {0x1A, 0xF8};
    const auto value = Obd::decodePid(0x0C, data, sizeof(data));
    TEST_ASSERT_TRUE(value.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1726.0f, value.value);
}

void test_pid_speed() {
    const uint8_t data[] = {88};
    const auto value = Obd::decodePid(0x0D, data, sizeof(data));
    TEST_ASSERT_TRUE(value.valid);
    TEST_ASSERT_EQUAL_FLOAT(88.0f, value.value);
}

void test_pid_coolant() {
    const uint8_t data[] = {130};
    const auto value = Obd::decodePid(0x05, data, sizeof(data));
    TEST_ASSERT_TRUE(value.valid);
    TEST_ASSERT_EQUAL_FLOAT(90.0f, value.value);
}

void test_pid_control_voltage() {
    const uint8_t data[] = {0x30, 0x39};
    const auto value = Obd::decodePid(0x42, data, sizeof(data));
    TEST_ASSERT_TRUE(value.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.345f, value.value);
}

void test_pid_invalid_length() {
    const uint8_t data[] = {0x12};
    TEST_ASSERT_FALSE(Obd::decodePid(0x0C, data, sizeof(data)).valid);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_pid_rpm);
    RUN_TEST(test_pid_speed);
    RUN_TEST(test_pid_coolant);
    RUN_TEST(test_pid_control_voltage);
    RUN_TEST(test_pid_invalid_length);
    return UNITY_END();
}
