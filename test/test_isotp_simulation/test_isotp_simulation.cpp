#include <unity.h>
#include "IsoTpSimulation.h"
#include "IsoTpReassembler.h"

void setUp() {}
void tearDown() {}

void test_normal_single_frame_sequence() {
    const auto sequence = Simulation::buildIsoTpSequence(Simulation::Scenario::NormalSingleFrame);
    TEST_ASSERT_EQUAL_UINT(1, sequence.frameCount);
    TEST_ASSERT_FALSE(sequence.flowControlExpected);
    TEST_ASSERT_FALSE(sequence.timeoutExpected);

    IsoTp::IsoTpReassembler reassembler;
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::Complete),
                      static_cast<int>(reassembler.processFrame(sequence.frames[0])));
}

void test_multiframe_vin_sequence_reassembles() {
    const auto sequence = Simulation::buildIsoTpSequence(Simulation::Scenario::NormalMultiFrameVin);
    TEST_ASSERT_GREATER_THAN_UINT(1, sequence.frameCount);
    TEST_ASSERT_TRUE(sequence.flowControlExpected);

    IsoTp::IsoTpReassembler reassembler;
    IsoTp::Status status = IsoTp::Status::Idle;
    for (std::size_t index = 0; index < sequence.frameCount; ++index) {
        status = reassembler.processFrame(sequence.frames[index]);
    }

    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::Complete), static_cast<int>(status));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT(17, reassembler.payload().length);
    TEST_ASSERT_EQUAL_UINT8(0x49, reassembler.payload().bytes[0]);
    TEST_ASSERT_EQUAL_UINT8(0x02, reassembler.payload().bytes[1]);
}

void test_dtc_sequence_reassembles() {
    const auto sequence = Simulation::buildIsoTpSequence(Simulation::Scenario::NormalMultiFrameDtc);
    IsoTp::IsoTpReassembler reassembler;
    IsoTp::Status status = IsoTp::Status::Idle;
    for (std::size_t index = 0; index < sequence.frameCount; ++index) {
        status = reassembler.processFrame(sequence.frames[index]);
    }
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::Complete), static_cast<int>(status));
    TEST_ASSERT_EQUAL_UINT8(0x43, reassembler.payload().bytes[0]);
}

void test_sequence_error_scenario() {
    const auto sequence = Simulation::buildIsoTpSequence(Simulation::Scenario::SequenceError);
    IsoTp::IsoTpReassembler reassembler;
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::InProgress),
                      static_cast<int>(reassembler.processFrame(sequence.frames[0])));
    TEST_ASSERT_EQUAL(static_cast<int>(IsoTp::Status::SequenceError),
                      static_cast<int>(reassembler.processFrame(sequence.frames[1])));
}

void test_timeout_and_negative_response_flags() {
    const auto timeoutSequence = Simulation::buildIsoTpSequence(Simulation::Scenario::TimeoutAfterFirstFrame);
    TEST_ASSERT_TRUE(timeoutSequence.timeoutExpected);
    TEST_ASSERT_EQUAL_UINT(1, timeoutSequence.frameCount);

    const auto negativeSequence = Simulation::buildIsoTpSequence(Simulation::Scenario::NegativeResponse);
    TEST_ASSERT_TRUE(negativeSequence.negativeResponse);
    TEST_ASSERT_EQUAL_UINT8(0x7F, negativeSequence.frames[0].data[1]);
}

void test_multiple_ecus_response() {
    const auto sequence = Simulation::buildIsoTpSequence(Simulation::Scenario::MultipleEcusResponse);
    TEST_ASSERT_EQUAL_UINT(2, sequence.frameCount);
    TEST_ASSERT_EQUAL_UINT32(0x7E8, sequence.frames[0].id);
    TEST_ASSERT_EQUAL_UINT32(0x7E9, sequence.frames[1].id);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_normal_single_frame_sequence);
    RUN_TEST(test_multiframe_vin_sequence_reassembles);
    RUN_TEST(test_dtc_sequence_reassembles);
    RUN_TEST(test_sequence_error_scenario);
    RUN_TEST(test_timeout_and_negative_response_flags);
    RUN_TEST(test_multiple_ecus_response);
    return UNITY_END();
}
