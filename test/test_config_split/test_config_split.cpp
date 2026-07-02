#include <cstring>

#include <unity.h>

#include "config/ProjectConfig.h"
#include "config/SenderConfig.h"
#include "config/DisplayConfig.h"
#include "config/NetworkConfig.h"
#include "config/SecurityConfig.h"
#include "config/SimulationConfig.h"
#include "config/LoggingConfig.h"
#include "config/ObdConfig.h"

void setUp() {}
void tearDown() {}

void test_project_config_has_version() {
    TEST_ASSERT_NOT_NULL(ProjectConfig::FirmwareVersion);
    TEST_ASSERT_GREATER_THAN(0U, std::strlen(ProjectConfig::FirmwareVersion));
    TEST_ASSERT_GREATER_THAN(0, ProjectConfig::ProtocolVersion);
}

void test_sender_config_has_valid_pins() {
    TEST_ASSERT_GREATER_OR_EQUAL(0, SenderConfig::CanRxPin);
    TEST_ASSERT_GREATER_OR_EQUAL(0, SenderConfig::CanTxPin);
    TEST_ASSERT_GREATER_OR_EQUAL(0, SenderConfig::ButtonPin);
    TEST_ASSERT_GREATER_THAN(0U, SenderConfig::CanRxQueueLength);
}

void test_display_config_page_bounds() {
    TEST_ASSERT_GREATER_THAN(0, DisplayConfig::PageCount);
    TEST_ASSERT_LESS_THAN(DisplayConfig::PageCount, DisplayConfig::MainPageIndex);
}

void test_network_config_esp_now() {
    TEST_ASSERT_GREATER_THAN(0, NetworkConfig::EspNowChannel);
    TEST_ASSERT_LESS_OR_EQUAL(14, NetworkConfig::EspNowChannel);
    TEST_ASSERT_NOT_NULL(NetworkConfig::DisplayPeerMac);
    TEST_ASSERT_NOT_NULL(NetworkConfig::SenderAllowedMac);
}

void test_simulation_is_off_by_default() {
    TEST_ASSERT_FALSE(SimulationConfig::EnableSimulationByDefault);
}

void test_obd_requested_pids_not_empty() {
    TEST_ASSERT_GREATER_THAN(0U, ObdConfig::ObdPidCount);
    TEST_ASSERT_NOT_EQUAL(0, ObdConfig::RequestedPids[0]);
}

void test_config_h_removed_from_include_path() {
#if __has_include("Config.h")
    TEST_FAIL_MESSAGE("include/Config.h must not exist or be reachable");
#else
    TEST_PASS();
#endif
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(test_project_config_has_version);
    RUN_TEST(test_sender_config_has_valid_pins);
    RUN_TEST(test_display_config_page_bounds);
    RUN_TEST(test_network_config_esp_now);
    RUN_TEST(test_simulation_is_off_by_default);
    RUN_TEST(test_obd_requested_pids_not_empty);
    RUN_TEST(test_config_h_removed_from_include_path);
    return UNITY_END();
}
