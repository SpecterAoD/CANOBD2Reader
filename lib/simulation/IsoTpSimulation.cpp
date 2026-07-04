#include "IsoTpSimulation.h"
#include <initializer_list>

namespace Simulation {

namespace {
IsoTp::CanFrame frame(uint32_t id, uint8_t dlc, std::initializer_list<uint8_t> bytes) {
    IsoTp::CanFrame out{};
    out.id = id;
    out.dlc = dlc;
    uint8_t index = 0;
    for (uint8_t value : bytes) {
        if (index >= sizeof(out.data)) break;
        out.data[index++] = value;
    }
    return out;
}
}

IsoTpSimulationSequence buildIsoTpSequence(Scenario scenario) {
    IsoTpSimulationSequence sequence{};
    switch (scenario) {
        case Scenario::NormalSingleFrame:
            sequence.frames[0] = frame(0x7E8, 4, {0x03, 0x41, 0x0D, 0x64});
            sequence.frameCount = 1;
            break;
        case Scenario::NormalMultiFrameVin:
        case Scenario::FlowControlRequired:
            sequence.frames[0] = frame(0x7E8, 8, {0x10, 0x14, 0x49, 0x02, 0x01, 'W', 'V', 'W'});
            sequence.frames[1] = frame(0x7E8, 8, {0x21, 'Z', 'Z', 'Z', '1', 'J', 'Z', 'X'});
            sequence.frames[2] = frame(0x7E8, 8, {0x22, 'W', '0', '0', '0', '0', '0', '1'});
            sequence.frameCount = 3;
            sequence.flowControlExpected = true;
            break;
        case Scenario::NormalMultiFrameDtc:
            sequence.frames[0] = frame(0x7E8, 8, {0x10, 0x08, 0x43, 0x01, 0x33, 0x04, 0x20, 0x00});
            sequence.frames[1] = frame(0x7E8, 3, {0x21, 0x00, 0x00});
            sequence.frameCount = 2;
            sequence.flowControlExpected = true;
            break;
        case Scenario::TimeoutAfterFirstFrame:
            sequence.frames[0] = frame(0x7E8, 8, {0x10, 0x11, 0x49, 0x02, 0x01, 'W', 'V', 'W'});
            sequence.frameCount = 1;
            sequence.flowControlExpected = true;
            sequence.timeoutExpected = true;
            break;
        case Scenario::SequenceError:
            sequence.frames[0] = frame(0x7E8, 8, {0x10, 0x08, 0x49, 0x02, 0x01, 'A', 'B', 'C'});
            sequence.frames[1] = frame(0x7E8, 8, {0x22, 'D', 'E', 'F', 'G', 'H', 'I', 'J'});
            sequence.frameCount = 2;
            sequence.flowControlExpected = true;
            break;
        case Scenario::BufferOverflow:
            // Classic ISO-TP FF length field is 12 bit, so 4095 is the maximum
            // encodable length. This case intentionally marks the scenario for
            // UI/test handling even though the raw FF bytes are max-length.
            sequence.frames[0] = frame(0x7E8, 8, {0x1F, 0xFF, 0, 1, 2, 3, 4, 5});
            sequence.frameCount = 1;
            sequence.flowControlExpected = true;
            break;
        case Scenario::MultipleEcusResponse:
            sequence.frames[0] = frame(0x7E8, 4, {0x03, 0x41, 0x0D, 0x64});
            sequence.frames[1] = frame(0x7E9, 4, {0x03, 0x41, 0x0C, 0x2E});
            sequence.frameCount = 2;
            break;
        case Scenario::NegativeResponse:
            sequence.frames[0] = frame(0x7E8, 4, {0x03, 0x7F, 0x01, 0x11});
            sequence.frameCount = 1;
            sequence.negativeResponse = true;
            break;
        case Scenario::DisplayNormalValues:
        case Scenario::DisplayWarningValues:
        case Scenario::DisplayCriticalValues:
        case Scenario::DisplayTimeoutValues:
        case Scenario::DisplayMixedValues:
        case Scenario::PowerRunning:
        case Scenario::PowerStartStop:
        case Scenario::PowerIdle:
        case Scenario::PowerParked:
        case Scenario::PowerDisplaySleep:
        case Scenario::PowerWakeup:
            sequence.frames[0] = frame(0x7E8, 4, {0x03, 0x41, 0x0D, 0x64});
            sequence.frameCount = 1;
            sequence.timeoutExpected = scenario == Scenario::DisplayTimeoutValues;
            break;
    }
    return sequence;
}

}
