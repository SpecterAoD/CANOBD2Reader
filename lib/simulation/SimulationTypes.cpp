#include "SimulationTypes.h"
#include <cstring>

namespace Simulation {

bool parseScenario(const char* text, Scenario& out) {
    if (text == nullptr) return false;
    for (uint8_t index = 0; index <= static_cast<uint8_t>(Scenario::PowerWakeup); ++index) {
        const auto candidate = static_cast<Scenario>(index);
        if (std::strcmp(text, scenarioName(candidate)) == 0) {
            out = candidate;
            return true;
        }
    }
    return false;
}

}
