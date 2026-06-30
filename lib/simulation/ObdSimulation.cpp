#include "ObdSimulation.h"
#include "simulation_data.h"
#include <cstring>

namespace Simulation {

namespace {

float displayScenarioValue(const SimulationData::Sample& sample, Scenario scenario) {
    const bool normal = scenario == Scenario::DisplayNormalValues;
    const bool warning = scenario == Scenario::DisplayWarningValues;
    const bool critical = scenario == Scenario::DisplayCriticalValues;
    const bool mixed = scenario == Scenario::DisplayMixedValues;

    if (std::strcmp(sample.name, "Speed") == 0) return mixed ? 72.0f : 88.0f;
    if (std::strcmp(sample.name, "RPM") == 0) {
        if (critical) return 5100.0f;
        if (warning || mixed) return 4300.0f;
        return normal ? 2500.0f : 2200.0f;
    }
    if (std::strcmp(sample.name, "CoolantTemp") == 0) {
        if (critical || mixed) return 108.0f;
        if (warning) return 98.0f;
        return 85.0f;
    }
    if (std::strcmp(sample.name, "BatteryVoltage") == 0) {
        if (critical) return 11.0f;
        if (warning) return 11.7f;
        if (mixed) return 13.8f;
        return 13.8f;
    }
    if (std::strcmp(sample.name, "OilTemp") == 0) {
        if (critical) return 128.0f;
        if (warning || mixed) return 116.0f;
        return 90.0f;
    }
    if (std::strcmp(sample.name, "EngineLoad") == 0) return warning ? 74.0f : 36.0f;
    if (std::strcmp(sample.name, "IntakeTemp") == 0) return 29.0f;
    if (std::strcmp(sample.name, "AverageConsumption") == 0) return 7.4f;
    if (std::strcmp(sample.name, "FuelRate") == 0) return 4.8f;
    if (std::strcmp(sample.name, "Throttle") == 0) return warning ? 72.0f : 22.0f;
    if (std::strcmp(sample.name, "MAF") == 0) return 14.6f;
    if (std::strcmp(sample.name, "FuelLevel") == 0) return 64.0f;
    if (std::strcmp(sample.name, "RunTime") == 0) return 845.0f;
    if (std::strcmp(sample.name, "AmbientTemp") == 0) return 24.0f;
    return 0.0f;
}

bool displayScenarioTimedOut(const SimulationData::Sample& sample, Scenario scenario) {
    if (scenario == Scenario::DisplayTimeoutValues) return true;
    return scenario == Scenario::DisplayMixedValues && std::strcmp(sample.name, "BatteryVoltage") == 0;
}

}

SimulatedPidValue simulatedPidValue(std::size_t index, unsigned long nowMs, Scenario scenario) {
    const SimulationData::Sample& sample = SimulationData::Samples[index % SimulationData::SampleCount];
    const float base = SimulationData::valueForSample(sample, nowMs, index);
    const char* status = "OK";
    float value = base;

    if (scenario == Scenario::DisplayNormalValues ||
        scenario == Scenario::DisplayWarningValues ||
        scenario == Scenario::DisplayCriticalValues ||
        scenario == Scenario::DisplayMixedValues) {
        value = displayScenarioValue(sample, scenario);
    }

    if (displayScenarioTimedOut(sample, scenario)) {
        status = "TIMEOUT";
    } else if (scenario == Scenario::NegativeResponse && index % 5 == 0) {
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
        case Scenario::DisplayNormalValues: return "Display color test: all values OK";
        case Scenario::DisplayWarningValues: return "Display color test: warning values";
        case Scenario::DisplayCriticalValues: return "Display color test: critical values";
        case Scenario::DisplayTimeoutValues: return "Display color test: timeout values";
        case Scenario::DisplayMixedValues: return "Display color test: mixed severities";
    }
    return "Unknown";
}

}
