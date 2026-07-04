#include <unity.h>
#include "AuthHelpers.h"
#include "WebRuntimeHandlers.h"
#include "config/ProjectConfig.h"
#include "config/SecurityConfig.h"

void setUp() {}
void tearDown() {}

void test_constant_time_equals() {
    TEST_ASSERT_TRUE(WebSecurity::constantTimeEquals("secret", "secret"));
    TEST_ASSERT_FALSE(WebSecurity::constantTimeEquals("secret", "Secret"));
    TEST_ASSERT_FALSE(WebSecurity::constantTimeEquals("secret", "secret1"));
    TEST_ASSERT_FALSE(WebSecurity::constantTimeEquals(nullptr, "secret"));
}

void test_authentication_config_is_present() {
    TEST_ASSERT_TRUE(SecurityConfig::EnableAuthentication);
    TEST_ASSERT_NOT_NULL(WebSecurity::username());
    TEST_ASSERT_NOT_NULL(WebSecurity::password());
    TEST_ASSERT_TRUE(WebSecurity::username()[0] != '\0');
    TEST_ASSERT_TRUE(WebSecurity::password()[0] != '\0');
}

void test_api_token_config_is_present_and_checked() {
    TEST_ASSERT_NOT_NULL(WebSecurity::apiToken());
    TEST_ASSERT_TRUE(WebSecurity::apiToken()[0] != '\0');
    TEST_ASSERT_TRUE(WebSecurity::isConfiguredToken(WebSecurity::apiToken()));
    TEST_ASSERT_FALSE(WebSecurity::isConfiguredToken("wrong-token"));
    TEST_ASSERT_FALSE(WebSecurity::isConfiguredToken(""));
    TEST_ASSERT_FALSE(WebSecurity::isConfiguredToken(nullptr));
}

void test_placeholder_secret_guard_mode_is_explicit() {
    TEST_ASSERT_FALSE(SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets);
    TEST_ASSERT_EQUAL(0, WebSecurity::senderManagementSecurityWarning().length());
    TEST_ASSERT_EQUAL(0, WebSecurity::displayManagementSecurityWarning().length());
    TEST_ASSERT_EQUAL(0, WebSecurity::espNowSecurityWarning().length());
    TEST_ASSERT_TRUE(WebSecurity::senderManagementConfigurationSafe());
    TEST_ASSERT_TRUE(WebSecurity::displayManagementConfigurationSafe());
    TEST_ASSERT_TRUE(WebSecurity::espNowConfigurationSafe());
}

void test_ota_target_filename_guard() {
    TEST_ASSERT_TRUE(SecurityConfig::RequireOtaTargetInFilename);

    TEST_ASSERT_TRUE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("sender.bin", "sender"));
    TEST_ASSERT_TRUE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("CANOBD2_sender_V1.0.11.bin", "sender"));
    TEST_ASSERT_FALSE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("display.bin", "sender"));
    TEST_ASSERT_FALSE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("firmware.bin", "sender"));

    TEST_ASSERT_TRUE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("display.bin", "display"));
    TEST_ASSERT_TRUE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("CANOBD2_display_V1.0.11.bin", "display"));
    TEST_ASSERT_FALSE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("sender.bin", "display"));
    TEST_ASSERT_FALSE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("", "display"));
    TEST_ASSERT_FALSE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("resender.bin", "sender"));
    TEST_ASSERT_FALSE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("display_senderish.bin", "sender"));
    TEST_ASSERT_FALSE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("sender.txt", "sender"));
    TEST_ASSERT_FALSE(WebRuntimeHandlers::firmwareFilenameMatchesTarget("../sender.bin", "sender"));
}

void test_ota_metadata_markers_can_be_found_in_binary_data() {
    String image = String("....sender....") + ProjectConfig::FirmwareVersion + "....";
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(image.c_str());
    const size_t size = image.length();

    TEST_ASSERT_TRUE(WebRuntimeHandlers::firmwareBufferContainsText(bytes, size, "sender"));
    TEST_ASSERT_TRUE(WebRuntimeHandlers::firmwareBufferContainsTargetMarker(bytes, size, "sender"));
    TEST_ASSERT_TRUE(WebRuntimeHandlers::firmwareBufferContainsVersionMarker(bytes, size, ProjectConfig::FirmwareVersion));
    TEST_ASSERT_FALSE(WebRuntimeHandlers::firmwareBufferContainsTargetMarker(bytes, size, "display"));
    TEST_ASSERT_FALSE(WebRuntimeHandlers::firmwareBufferContainsVersionMarker(bytes, size, "V9.9.9"));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_constant_time_equals);
    RUN_TEST(test_authentication_config_is_present);
    RUN_TEST(test_api_token_config_is_present_and_checked);
    RUN_TEST(test_placeholder_secret_guard_mode_is_explicit);
    RUN_TEST(test_ota_target_filename_guard);
    RUN_TEST(test_ota_metadata_markers_can_be_found_in_binary_data);
    return UNITY_END();
}
