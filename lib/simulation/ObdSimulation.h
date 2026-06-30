#pragma once

#include <cstddef>
#include <cstdint>
#include "SimulationTypes.h"

namespace Simulation {

struct SimulatedPidValue {
    const char* type;
    const char* key;
    const char* name;
    float value;
    const char* unit;
    const char* status;
    uint8_t decimals;
};

SimulatedPidValue simulatedPidValue(std::size_t index, unsigned long nowMs, Scenario scenario);
std::size_t simulatedPidCount();
const char* scenarioDiagnosticText(Scenario scenario);

}
