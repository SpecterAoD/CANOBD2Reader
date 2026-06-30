#pragma once

#include <cstdint>

namespace Simulation {

enum class Scenario : uint8_t {
    NormalSingleFrame,
    NormalMultiFrameVin,
    NormalMultiFrameDtc,
    FlowControlRequired,
    TimeoutAfterFirstFrame,
    SequenceError,
    BufferOverflow,
    MultipleEcusResponse,
    NegativeResponse,
    DisplayNormalValues,
    DisplayWarningValues,
    DisplayCriticalValues,
    DisplayTimeoutValues,
    DisplayMixedValues
};

constexpr const char* scenarioName(Scenario scenario) {
    switch (scenario) {
        case Scenario::NormalSingleFrame: return "NormalSingleFrame";
        case Scenario::NormalMultiFrameVin: return "NormalMultiFrameVin";
        case Scenario::NormalMultiFrameDtc: return "NormalMultiFrameDtc";
        case Scenario::FlowControlRequired: return "FlowControlRequired";
        case Scenario::TimeoutAfterFirstFrame: return "TimeoutAfterFirstFrame";
        case Scenario::SequenceError: return "SequenceError";
        case Scenario::BufferOverflow: return "BufferOverflow";
        case Scenario::MultipleEcusResponse: return "MultipleEcusResponse";
        case Scenario::NegativeResponse: return "NegativeResponse";
        case Scenario::DisplayNormalValues: return "DisplayNormalValues";
        case Scenario::DisplayWarningValues: return "DisplayWarningValues";
        case Scenario::DisplayCriticalValues: return "DisplayCriticalValues";
        case Scenario::DisplayTimeoutValues: return "DisplayTimeoutValues";
        case Scenario::DisplayMixedValues: return "DisplayMixedValues";
    }
    return "NormalSingleFrame";
}

bool parseScenario(const char* text, Scenario& out);

}
