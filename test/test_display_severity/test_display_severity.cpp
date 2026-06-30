#include <unity.h>
#include "DisplaySeverity.h"

using DisplayLogic::DisplaySeverity;

void setUp() {}
void tearDown() {}

void test_coolant_severity() {
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Ok),
                      static_cast<int>(DisplayLogic::severityForMetric("CoolantTemp", 85.0f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(DisplayLogic::severityForMetric("CoolantTemp", 98.0f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(DisplayLogic::severityForMetric("CoolantTemp", 108.0f, true)));
}

void test_oil_severity() {
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Ok),
                      static_cast<int>(DisplayLogic::severityForMetric("OilTemp", 90.0f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(DisplayLogic::severityForMetric("OilTemp", 116.0f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(DisplayLogic::severityForMetric("OilTemp", 128.0f, true)));
}

void test_voltage_severity() {
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Ok),
                      static_cast<int>(DisplayLogic::severityForMetric("BatteryVoltage", 13.8f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(DisplayLogic::severityForMetric("BatteryVoltage", 11.7f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(DisplayLogic::severityForMetric("BatteryVoltage", 11.0f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(DisplayLogic::severityForMetric("BatteryVoltage", 15.0f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(DisplayLogic::severityForMetric("BatteryVoltage", 15.3f, true)));
}

void test_rpm_severity() {
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Ok),
                      static_cast<int>(DisplayLogic::severityForMetric("RPM", 2500.0f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(DisplayLogic::severityForMetric("RPM", 4300.0f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(DisplayLogic::severityForMetric("RPM", 5100.0f, true)));
}

void test_boost_severity() {
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Ok),
                      static_cast<int>(DisplayLogic::severityForMetric("BoostPressureBar", 0.19f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Warning),
                      static_cast<int>(DisplayLogic::severityForMetric("BoostPressureBar", 0.89f, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Critical),
                      static_cast<int>(DisplayLogic::severityForMetric("BoostPressureBar", 1.49f, true)));
}

void test_timeout_and_unknown_severity() {
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Timeout),
                      static_cast<int>(DisplayLogic::severityForMetric("CoolantTemp", 85.0f, false)));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Timeout),
                      static_cast<int>(DisplayLogic::severityForMetric("CoolantTemp", 85.0f, true, "TIMEOUT")));
    TEST_ASSERT_EQUAL(static_cast<int>(DisplaySeverity::Unknown),
                      static_cast<int>(DisplayLogic::severityForMetric("UnknownMetric", 123.0f, true)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_coolant_severity);
    RUN_TEST(test_oil_severity);
    RUN_TEST(test_voltage_severity);
    RUN_TEST(test_rpm_severity);
    RUN_TEST(test_boost_severity);
    RUN_TEST(test_timeout_and_unknown_severity);
    return UNITY_END();
}
