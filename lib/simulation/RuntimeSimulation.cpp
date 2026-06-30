#include "RuntimeSimulation.h"
#include "SimulationConfig.h"

namespace Simulation {

namespace {
bool enabledState = SimulationConfig::EnableSimulationByDefault;
Scenario scenarioState = SimulationConfig::DefaultScenario;
}

void RuntimeSimulation::resetForBoot() {
    enabledState = SimulationConfig::EnableSimulationByDefault;
    scenarioState = SimulationConfig::DefaultScenario;
}

bool RuntimeSimulation::enabled() {
    return enabledState;
}

void RuntimeSimulation::setEnabled(bool value) {
    enabledState = value;
}

bool RuntimeSimulation::toggle() {
    enabledState = !enabledState;
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
