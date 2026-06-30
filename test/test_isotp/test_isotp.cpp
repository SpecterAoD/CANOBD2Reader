#include <unity.h>
#include "IsoTpReassembler.h"

void setUp() {}
void tearDown() {}

void test_isotp_single_frame() {
    IsoTp::IsoTpReassembler reassembler;
    IsoTp::CanFrame frame{0x7E8, 4, {0x03, 0x41, 0x0D, 0x64}};
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::Complete), static_cast<int>(reassembler.processFrame(frame)));
    TEST_ASSERT_EQUAL_UINT(3, reassembler.payload().length);
    TEST_ASSERT_EQUAL_UINT8(0x41, reassembler.payload().bytes[0]);
    TEST_ASSERT_EQUAL_UINT8(0x64, reassembler.payload().bytes[2]);
}

void test_isotp_first_and_consecutive_frame() {
    IsoTp::IsoTpReassembler reassembler;
    IsoTp::CanFrame first{0x7E8, 8, {0x10, 0x11, 0x49, 0x02, 0x01, 'W', 'V', 'W'}};
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::InProgress), static_cast<int>(reassembler.processFrame(first)));
    IsoTp::CanFrame cf1{0x7E8, 8, {0x21, 'Z', 'Z', 'Z', '1', 'J', 'Z', 'X'}};
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::InProgress), static_cast<int>(reassembler.processFrame(cf1)));
    IsoTp::CanFrame cf2{0x7E8, 5, {0x22, 'W', '0', '0', '0'}};
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::Complete), static_cast<int>(reassembler.processFrame(cf2)));
    TEST_ASSERT_EQUAL_UINT(17, reassembler.payload().length);
}

void test_isotp_sequence_error() {
    IsoTp::IsoTpReassembler reassembler;
    IsoTp::CanFrame first{0x7E8, 8, {0x10, 0x08, 0x49, 0x02, 0x01, 'A', 'B', 'C'}};
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::InProgress), static_cast<int>(reassembler.processFrame(first)));
    IsoTp::CanFrame wrong{0x7E8, 8, {0x22, 'D', 'E', 'F', 'G', 'H', 'I', 'J'}};
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::SequenceError), static_cast<int>(reassembler.processFrame(wrong)));
}

void test_isotp_accepts_maximum_first_frame_length() {
    IsoTp::IsoTpReassembler reassembler;
    IsoTp::CanFrame first{0x7E8, 8, {0x1F, 0xFF, 0, 1, 2, 3, 4, 5}};
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::InProgress), static_cast<int>(reassembler.processFrame(first)));
    TEST_ASSERT_EQUAL_UINT(6, reassembler.payload().length);
}

void test_isotp_rejects_invalid_first_frame_length() {
    IsoTp::IsoTpReassembler reassembler;
    IsoTp::CanFrame first{0x7E8, 8, {0x10, 0x06, 0, 1, 2, 3, 4, 5}};
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::InvalidLength), static_cast<int>(reassembler.processFrame(first)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_isotp_single_frame);
    RUN_TEST(test_isotp_first_and_consecutive_frame);
    RUN_TEST(test_isotp_sequence_error);
    RUN_TEST(test_isotp_accepts_maximum_first_frame_length);
    RUN_TEST(test_isotp_rejects_invalid_first_frame_length);
    return UNITY_END();
}
