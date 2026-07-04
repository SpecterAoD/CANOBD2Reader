#include <unity.h>

#include <cstddef>
#include <cstring>

#include "ObdSimulation.h"
#include "SimulationTypes.h"

namespace {

bool simulationProvidesMetric(const char* metricName, Simulation::Scenario scenario) {
    for (std::size_t index = 0; index < Simulation::simulatedPidCount(); ++index) {
        const auto sample = Simulation::simulatedPidValue(index, 1000UL, scenario);
        if (std::strcmp(sample.name, metricName) == 0) {
            return std::strcmp(sample.status, "OK") == 0 || std::strcmp(sample.status, "WARN") == 0;
        }
    }
    return false;
}

void assertMetricAvailable(const char* metricName) {
    TEST_ASSERT_TRUE_MESSAGE(simulationProvidesMetric(metricName, Simulation::Scenario::DisplayNormalValues), metricName);
}

const char* expectedPowerState(Simulation::Scenario scenario) {
    switch (scenario) {
        case Simulation::Scenario::PowerRunning: return "Running";
        case Simulation::Scenario::PowerStartStop: return "StartStop";
        case Simulation::Scenario::PowerIdle: return "Idle";
        case Simulation::Scenario::PowerParked: return "Parked";
        case Simulation::Scenario::PowerDisplaySleep: return "DisplaySleep";
        case Simulation::Scenario::PowerWakeup: return "Running";
        default: return "Running";
    }
}

const char* expectedPowerCommand(Simulation::Scenario scenario) {
    if (scenario == Simulation::Scenario::PowerDisplaySleep) return "Sleep";
    if (scenario == Simulation::Scenario::PowerWakeup) return "Wakeup";
    return "None";
}

} // namespace

void setUp() {}
void tearDown() {}

void test_sender_simulation_covers_main_engine_trip_and_additional_pages() {
    assertMetricAvailable("Speed");
    assertMetricAvailable("RPM");
    assertMetricAvailable("CoolantTemp");
    assertMetricAvailable("BatteryVoltage");
    assertMetricAvailable("OilTemp");
    assertMetricAvailable("EngineLoad");
    assertMetricAvailable("IntakeTemp");
    assertMetricAvailable("AverageConsumption");
    assertMetricAvailable("FuelRate");
    assertMetricAvailable("Throttle");
    assertMetricAvailable("MAF");
    assertMetricAvailable("FuelLevel");
    assertMetricAvailable("RunTime");
    assertMetricAvailable("AmbientTemp");
    assertMetricAvailable("ManifoldAbsolutePressure");
    assertMetricAvailable("BarometricPressure");
}

void test_sender_simulation_power_scenarios_match_display_expectations() {
    TEST_ASSERT_EQUAL_STRING("Running", expectedPowerState(Simulation::Scenario::PowerRunning));
    TEST_ASSERT_EQUAL_STRING("StartStop", expectedPowerState(Simulation::Scenario::PowerStartStop));
    TEST_ASSERT_EQUAL_STRING("Idle", expectedPowerState(Simulation::Scenario::PowerIdle));
    TEST_ASSERT_EQUAL_STRING("Parked", expectedPowerState(Simulation::Scenario::PowerParked));
    TEST_ASSERT_EQUAL_STRING("DisplaySleep", expectedPowerState(Simulation::Scenario::PowerDisplaySleep));
    TEST_ASSERT_EQUAL_STRING("Running", expectedPowerState(Simulation::Scenario::PowerWakeup));

    TEST_ASSERT_EQUAL_STRING("Sleep", expectedPowerCommand(Simulation::Scenario::PowerDisplaySleep));
    TEST_ASSERT_EQUAL_STRING("Wakeup", expectedPowerCommand(Simulation::Scenario::PowerWakeup));
    TEST_ASSERT_EQUAL_STRING("None", expectedPowerCommand(Simulation::Scenario::PowerStartStop));
}

void test_sender_simulation_display_warning_and_critical_values_are_available() {
    TEST_ASSERT_TRUE(simulationProvidesMetric("RPM", Simulation::Scenario::DisplayWarningValues));
    TEST_ASSERT_TRUE(simulationProvidesMetric("CoolantTemp", Simulation::Scenario::DisplayWarningValues));
    TEST_ASSERT_TRUE(simulationProvidesMetric("RPM", Simulation::Scenario::DisplayCriticalValues));
    TEST_ASSERT_TRUE(simulationProvidesMetric("CoolantTemp", Simulation::Scenario::DisplayCriticalValues));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_sender_simulation_covers_main_engine_trip_and_additional_pages);
    RUN_TEST(test_sender_simulation_power_scenarios_match_display_expectations);
    RUN_TEST(test_sender_simulation_display_warning_and_critical_values_are_available);
    return UNITY_END();
}
