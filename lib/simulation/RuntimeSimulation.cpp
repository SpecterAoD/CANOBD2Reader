#include "RuntimeSimulation.h"

#include "common_config.h"
#include "config/SimulationConfig.h"

namespace Simulation {

namespace {
bool enabledState = SimulationConfig::EnableSimulationByDefault;
Scenario scenarioState = SimulationConfig::DefaultScenario;
}

void RuntimeSimulation::resetForBoot() {
    enabledState = CANOBD2_DEV_TOOLS && SimulationConfig::EnableSimulationByDefault;
    scenarioState = SimulationConfig::DefaultScenario;
}

bool RuntimeSimulation::enabled() {
    return enabledState;
}

// Release builds (CANOBD2_DEV_TOOLS=0) can never switch to simulated data.
void RuntimeSimulation::setEnabled(bool value) {
    enabledState = CANOBD2_DEV_TOOLS && value;
}

bool RuntimeSimulation::toggle() {
    setEnabled(!enabledState);
    return enabledState;
}

Scenario RuntimeSimulation::scenario() {
    return scenarioState;
}

void RuntimeSimulation::setScenario(Scenario value) {
    scenarioState = value;
}

const char* RuntimeSimulation::scenarioName() {
    return Simulation::scenarioName(scenarioState);
}

}
