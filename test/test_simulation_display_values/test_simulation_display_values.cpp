#include <unity.h>
#include <cstddef>
#include <cstring>
#include "DisplaySeverity.h"
#include "ObdSimulation.h"

using DisplayLogic::DisplaySeverity;

void setUp() {}
void tearDown() {}

DisplaySeverity simulatedSeverityFor(const char* metric, Simulation::Scenario scenario) {
    for (std::size_t index = 0; index < Simulation::simulatedPidCount(); ++index) {
        const auto sample = Simulation::simulatedPidValue(index, 1000UL, scenario);
        if (std::strcmp(sample.name, metric) == 0) {
            const bool fresh = std::strcmp(sample.status, "TIMEOUT") != 0;
            return DisplayLogic::severityForMetric(sample.name, sample.value, fresh, sample.status);
        }
    }
    return DisplaySeverity::Unknown;
}

void test_display_normal_values_are_ok() {
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Ok),
                      static_cast<int>(simulatedSeverityFor("CoolantTemp", Simulation::Scenario::DisplayNormalValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Ok),
                      static_cast<int>(simulatedSeverityFor("OilTemp", Simulation::Scenario::DisplayNormalValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Ok),
                      static_cast<int>(simulatedSeverityFor("BatteryVoltage", Simulation::Scenario::DisplayNormalValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Ok),
                      static_cast<int>(simulatedSeverityFor("RPM", Simulation::Scenario::DisplayNormalValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Ok),
                      static_cast<int>(simulatedSeverityFor("BoostPressureBar", Simulation::Scenario::DisplayNormalValues)));
}

void test_display_warning_values_are_warning() {
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(simulatedSeverityFor("CoolantTemp", Simulation::Scenario::DisplayWarningValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(simulatedSeverityFor("OilTemp", Simulation::Scenario::DisplayWarningValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(simulatedSeverityFor("BatteryVoltage", Simulation::Scenario::DisplayWarningValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(simulatedSeverityFor("RPM", Simulation::Scenario::DisplayWarningValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(simulatedSeverityFor("BoostPressureBar", Simulation::Scenario::DisplayWarningValues)));
}

void test_display_critical_values_are_critical() {
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(simulatedSeverityFor("CoolantTemp", Simulation::Scenario::DisplayCriticalValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(simulatedSeverityFor("OilTemp", Simulation::Scenario::DisplayCriticalValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(simulatedSeverityFor("BatteryVoltage", Simulation::Scenario::DisplayCriticalValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(simulatedSeverityFor("RPM", Simulation::Scenario::DisplayCriticalValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(simulatedSeverityFor("BoostPressureBar", Simulation::Scenario::DisplayCriticalValues)));
}

void test_display_timeout_values_are_timeout() {
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Timeout),
                      static_cast<int>(simulatedSeverityFor("CoolantTemp", Simulation::Scenario::DisplayTimeoutValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Timeout),
                      static_cast<int>(simulatedSeverityFor("BatteryVoltage", Simulation::Scenario::DisplayTimeoutValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Timeout),
                      static_cast<int>(simulatedSeverityFor("BoostPressureBar", Simulation::Scenario::DisplayTimeoutValues)));
}

void test_display_mixed_values_are_mixed() {
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Ok),
                      static_cast<int>(simulatedSeverityFor("Speed", Simulation::Scenario::DisplayMixedValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(simulatedSeverityFor("OilTemp", Simulation::Scenario::DisplayMixedValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(simulatedSeverityFor("CoolantTemp", Simulation::Scenario::DisplayMixedValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Timeout),
                      static_cast<int>(simulatedSeverityFor("BatteryVoltage", Simulation::Scenario::DisplayMixedValues)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(simulatedSeverityFor("BoostPressureBar", Simulation::Scenario::DisplayMixedValues)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_display_normal_values_are_ok);
    RUN_TEST(test_display_warning_values_are_warning);
    RUN_TEST(test_display_critical_values_are_critical);
    RUN_TEST(test_display_timeout_values_are_timeout);
    RUN_TEST(test_display_mixed_values_are_mixed);
    return UNITY_END();
}
