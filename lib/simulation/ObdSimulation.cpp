#include "ObdSimulation.h"
#include "simulation_data.h"

namespace Simulation {

SimulatedPidValue simulatedPidValue(std::size_t index, unsigned long nowMs, Scenario scenario) {
    const SimulationData::Sample& sample = SimulationData::Samples[index % SimulationData::SampleCount];
    const float base = SimulationData::valueForSample(sample, nowMs, index);
    const char* status = "OK";
    float value = base;

    if (scenario == Scenario::NegativeResponse && index % 5 == 0) {
        status = "ERROR";
        value = 0.0f;
    } else if (scenario == Scenario::TimeoutAfterFirstFrame && index % 4 == 0) {
        status = "TIMEOUT";
        value = 0.0f;
    }

    return {sample.type, sample.key, sample.name, value, sample.unit, status, sample.decimals};
}

std::size_t simulatedPidCount() {
    return SimulationData::SampleCount;
}

const char* scenarioDiagnosticText(Scenario scenario) {
    switch (scenario) {
        case Scenario::NormalSingleFrame: return "Single-frame OBD live data";
        case Scenario::NormalMultiFrameVin: return "Multi-frame VIN response";
        case Scenario::NormalMultiFrameDtc: return "Multi-frame DTC response";
        case Scenario::FlowControlRequired: return "Flow Control CTS required";
        case Scenario::TimeoutAfterFirstFrame: return "Timeout after First Frame";
        case Scenario::SequenceError: return "Consecutive Frame sequence error";
        case Scenario::BufferOverflow: return "ISO-TP max/overflow boundary";
        case Scenario::MultipleEcusResponse: return "Multiple ECU responses";
        case Scenario::NegativeResponse: return "OBD negative response 0x7F";
    }
    return "Unknown";
}

}
