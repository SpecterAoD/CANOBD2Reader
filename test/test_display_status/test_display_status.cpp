#include <unity.h>
#include "config/DisplayConfig.h"
#include "StatusLogic.h"

void setUp() {}
void tearDown() {}

void test_valid_heartbeat_sets_esp_now_ok() {
    TEST_ASSERT_EQUAL(static_cast<int>(StatusLogic::Health::Ok),
                      static_cast<int>(StatusLogic::packetLinkHealth(2000, 1000, DisplayConfig::EspNowTimeoutMs)));
}

void test_missing_packets_sets_esp_now_error() {
    TEST_ASSERT_EQUAL(static_cast<int>(StatusLogic::Health::Error),
                      static_cast<int>(StatusLogic::packetLinkHealth(8000, 1000, DisplayConfig::EspNowTimeoutMs)));
}

void test_missing_obd_does_not_break_esp_now() {
    TEST_ASSERT_EQUAL(static_cast<int>(StatusLogic::Health::Ok),
                      static_cast<int>(StatusLogic::packetLinkHealth(2000, 1000, DisplayConfig::EspNowTimeoutMs)));
    TEST_ASSERT_EQUAL(static_cast<int>(StatusLogic::Health::Warning),
                      static_cast<int>(StatusLogic::obdHealth(true, 2000, 0, DisplayConfig::ObdTimeoutMs)));
}

void test_can_and_obd_timeouts_are_independent() {
    TEST_ASSERT_EQUAL(static_cast<int>(StatusLogic::Health::Warning),
                      static_cast<int>(StatusLogic::canHealth(true, 9000, 1000, DisplayConfig::CanTimeoutMs)));
    TEST_ASSERT_EQUAL(static_cast<int>(StatusLogic::Health::Ok),
                      static_cast<int>(StatusLogic::obdHealth(true, 3000, 1000, DisplayConfig::ObdTimeoutMs)));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_valid_heartbeat_sets_esp_now_ok);
    RUN_TEST(test_missing_packets_sets_esp_now_error);
    RUN_TEST(test_missing_obd_does_not_break_esp_now);
    RUN_TEST(test_can_and_obd_timeouts_are_independent);
    return UNITY_END();
}
