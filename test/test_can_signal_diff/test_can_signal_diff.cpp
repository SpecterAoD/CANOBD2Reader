#include <unity.h>

#include "CanSignalDiff.h"

void setUp() {}
void tearDown() {}

void test_can_signal_diff_detects_changed_bytes_and_bits() {
    Capabilities::CanFrameSample before{};
    before.canId = 0x123;
    before.length = 4;
    before.data[0] = 0x00;
    before.data[1] = 0x55;
    before.data[2] = 0xAA;
    before.data[3] = 0xFF;

    Capabilities::CanFrameSample after = before;
    after.data[1] = 0x57;
    after.data[3] = 0x0F;

    Capabilities::CanSignalCandidate candidates[4]{};
    const std::size_t count = Capabilities::diffCanFrames(before, after, candidates, 4);
    TEST_ASSERT_EQUAL_UINT(2, count);
    TEST_ASSERT_EQUAL_UINT8(1, candidates[0].byteIndex);
    TEST_ASSERT_EQUAL_HEX8(0x02, candidates[0].changedBitMask);
    TEST_ASSERT_EQUAL_UINT8(3, candidates[1].byteIndex);
    TEST_ASSERT_EQUAL_HEX8(0xF0, candidates[1].changedBitMask);
}

void test_can_signal_diff_ignores_different_ids() {
    Capabilities::CanFrameSample before{};
    before.canId = 0x123;
    before.length = 1;
    before.data[0] = 0x00;
    Capabilities::CanFrameSample after = before;
    after.canId = 0x124;
    after.data[0] = 0xFF;
    Capabilities::CanSignalCandidate candidates[2]{};
    TEST_ASSERT_EQUAL_UINT(0, Capabilities::diffCanFrames(before, after, candidates, 2));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_can_signal_diff_detects_changed_bytes_and_bits);
    RUN_TEST(test_can_signal_diff_ignores_different_ids);
    return UNITY_END();
}
