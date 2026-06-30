#include <unity.h>
#include "RuntimeSimulation.h"
#include "SimulationTypes.h"

void setUp() {}
void tearDown() {}

void test_simulation_defaults_to_disabled_after_boot_reset() {
    Simulation::RuntimeSimulation::setEnabled(true);
    Simulation::RuntimeSimulation::setScenario(Simulation::Scenario::NegativeResponse);

    Simulation::RuntimeSimulation::resetForBoot();

    TEST_ASSERT_FALSE(Simulation::RuntimeSimulation::enabled());
    TEST_ASSERT_EQUAL(static_cast<int>(Simulation::Scenario::NormalSingleFrame),
                      static_cast<int>(Simulation::RuntimeSimulation::scenario()));
}

void test_simulation_toggle_is_runtime_only() {
    Simulation::RuntimeSimulation::resetForBoot();
    TEST_ASSERT_FALSE(Simulation::RuntimeSimulation::enabled());

    TEST_ASSERT_TRUE(Simulation::RuntimeSimulation::toggle());
    TEST_ASSERT_TRUE(Simulation::RuntimeSimulation::enabled());

    TEST_ASSERT_FALSE(Simulation::RuntimeSimulation::toggle());
    TEST_ASSERT_FALSE(Simulation::RuntimeSimulation::enabled());
}

void test_simulation_scenario_parse() {
    Simulation::Scenario scenario = Simulation::Scenario::NormalSingleFrame;
    TEST_ASSERT_TRUE(Simulation::parseScenario("NormalMultiFrameVin", scenario));
    TEST_ASSERT_EQUAL(static_cast<int>(Simulation::Scenario::NormalMultiFrameVin),
                      static_cast<int>(scenario));
    TEST_ASSERT_FALSE(Simulation::parseScenario("NotARealScenario", scenario));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_simulation_defaults_to_disabled_after_boot_reset);
    RUN_TEST(test_simulation_toggle_is_runtime_only);
    RUN_TEST(test_simulation_scenario_parse);
    return UNITY_END();
}
