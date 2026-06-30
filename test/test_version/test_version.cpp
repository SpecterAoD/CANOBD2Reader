#include <cstring>
#include <unity.h>

#include "ProjectConfig.h"
#include "common_config.h"

void setUp() {}
void tearDown() {}

void test_firmware_version_is_available() {
    TEST_ASSERT_NOT_NULL(ProjectConfig::FirmwareVersion);
    TEST_ASSERT_TRUE(std::strlen(ProjectConfig::FirmwareVersion) > 0);
}

void test_target_name_is_known_value() {
    TEST_ASSERT_NOT_NULL(ProjectConfig::TargetName);
    const bool valid = std::strcmp(ProjectConfig::TargetName, "sender") == 0 ||
                       std::strcmp(ProjectConfig::TargetName, "display") == 0 ||
                       std::strcmp(ProjectConfig::TargetName, "unknown") == 0;
    TEST_ASSERT_TRUE(valid);
}

void test_protocol_version_is_positive() {
    TEST_ASSERT_TRUE(ProjectConfig::ProtocolVersion > 0);
}

void test_ota_config_fallback_is_present() {
    TEST_ASSERT_EQUAL_INT(1, CANOBD2_OTA_COMPATIBLE);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_firmware_version_is_available);
    RUN_TEST(test_target_name_is_known_value);
    RUN_TEST(test_protocol_version_is_positive);
    RUN_TEST(test_ota_config_fallback_is_present);
    return UNITY_END();
}
