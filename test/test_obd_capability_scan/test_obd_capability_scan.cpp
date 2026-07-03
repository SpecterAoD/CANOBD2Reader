#include <unity.h>

#include "ObdCapability.h"

void setUp() {}
void tearDown() {}

void test_pid_masks_identify_supported_pids() {
    TEST_ASSERT_TRUE(Capabilities::isPidSupportedByMask(0x0C, 0x00100000UL, 0, 0));
    TEST_ASSERT_FALSE(Capabilities::isPidSupportedByMask(0x0D, 0x00100000UL, 0, 0));
    TEST_ASSERT_TRUE(Capabilities::isPidSupportedByMask(0x33, 0, 0x00002000UL, 0));
    TEST_ASSERT_TRUE(Capabilities::isPidSupportedByMask(0x5E, 0, 0, 0x00000004UL));
}

void test_pid_range_and_next_range_detection() {
    TEST_ASSERT_EQUAL_UINT8(0x00, Capabilities::supportedRangeBaseForPid(0x01));
    TEST_ASSERT_EQUAL_UINT8(0x20, Capabilities::supportedRangeBaseForPid(0x21));
    TEST_ASSERT_EQUAL_UINT8(0x40, Capabilities::supportedRangeBaseForPid(0x41));
    TEST_ASSERT_EQUAL_UINT8(0x60, Capabilities::supportedRangeBaseForPid(0x61));
    TEST_ASSERT_TRUE(Capabilities::maskAdvertisesNextRange(0x00, 0x00000001UL));
    TEST_ASSERT_FALSE(Capabilities::maskAdvertisesNextRange(0x00, 0x00000000UL));
}

void test_recommended_pid_descriptors() {
    std::size_t count = 0;
    const auto* pids = Capabilities::recommendedObdPids(count);
    TEST_ASSERT_NOT_NULL(pids);
    TEST_ASSERT_GREATER_OR_EQUAL(15U, count);
    const auto* rpm = Capabilities::findObdPidDescriptor(0x0C);
    TEST_ASSERT_NOT_NULL(rpm);
    TEST_ASSERT_EQUAL_STRING("RPM", rpm->name);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_pid_masks_identify_supported_pids);
    RUN_TEST(test_pid_range_and_next_range_detection);
    RUN_TEST(test_recommended_pid_descriptors);
    return UNITY_END();
}
