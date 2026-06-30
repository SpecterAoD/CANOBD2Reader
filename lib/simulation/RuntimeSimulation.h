#pragma once

#include "SimulationTypes.h"

namespace Simulation {

/**
 * Runtime-only simulation switch.
 *
 * The state intentionally lives only in RAM. It is not written to NVS, EEPROM,
 * SPIFFS or flash. Each firmware boot starts from SimulationConfig defaults.
 */
class RuntimeSimulation {
public:
    static void resetForBoot();
    static bool enabled();
    static void setEnabled(bool value);
    static bool toggle();
    static Scenario scenario();
    static void setScenario(Scenario value);
    static const char* scenarioName();
};

}
