// Device secrets, authentication helpers and OTA image validation.
#include <unity.h>

#include <cstring>
#include <string>

#include "AuthHelpers.h"
#include "DeviceSecrets.h"
#include "OtaImageValidator.h"

using Security::DeviceSecrets;

void setUp() {}
void tearDown() {}

namespace {
DeviceSecrets validSecrets() {
    DeviceSecrets s{};
    std::strcpy(s.senderApSsid, "CANOBD2_Sender");
    std::strcpy(s.senderApPassword, "sender-pass-123");
    std::strcpy(s.displayApSsid, "CANOBD2_Display");
    std::strcpy(s.displayApPassword, "display-pass-123");
    std::strcpy(s.webUser, "admin");
    std::strcpy(s.webPassword, "web-pass-1234");
    std::strcpy(s.apiToken, "0123456789abcdef0123");
    for (int i = 0; i < 16; ++i) s.espNowKey[i] = static_cast<uint8_t>(i * 13 + 7);
    const uint8_t display[6] = {0xF0, 0xF5, 0xBD, 0x01, 0x02, 0x03};
    const uint8_t sender[6] = {0x8C, 0x4B, 0x14, 0x04, 0x05, 0x06};
    std::memcpy(s.displayMac, display, 6);
    std::memcpy(s.senderMac, sender, 6);
    return s;
}
} // namespace

void test_constant_time_equals() {
    TEST_ASSERT_TRUE(WebSecurity::constantTimeEquals("secret", "secret"));
    TEST_ASSERT_FALSE(WebSecurity::constantTimeEquals("secret", "Secret"));
    TEST_ASSERT_FALSE(WebSecurity::constantTimeEquals("secret", "secret1"));
    TEST_ASSERT_FALSE(WebSecurity::constantTimeEquals(nullptr, "secret"));
}

void test_valid_secrets_pass_for_both_targets() {
    const DeviceSecrets s = validSecrets();
    char problems[256];
    TEST_ASSERT_TRUE_MESSAGE(Security::validateSecrets(s, true, problems, sizeof(problems)), problems);
    TEST_ASSERT_TRUE_MESSAGE(Security::validateSecrets(s, false, problems, sizeof(problems)), problems);
}

void test_placeholders_and_weak_values_are_rejected() {
    char problems[256];
    DeviceSecrets s = validSecrets();
    std::strcpy(s.webPassword, "change-me-web");
    TEST_ASSERT_FALSE(Security::validateSecrets(s, true, problems, sizeof(problems)));
    TEST_ASSERT_NOT_NULL(std::strstr(problems, "Web-Passwort"));

    s = validSecrets();
    std::strcpy(s.senderApPassword, "short");
    TEST_ASSERT_FALSE(Security::validateSecrets(s, true, problems, sizeof(problems)));

    s = validSecrets();
    std::memset(s.espNowKey, 0, sizeof(s.espNowKey));
    TEST_ASSERT_FALSE(Security::validateSecrets(s, true, problems, sizeof(problems)));

    // The key from the old public example file counts as placeholder.
    const uint8_t legacy[16] = {0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE,
                                0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01};
    s = validSecrets();
    std::memcpy(s.espNowKey, legacy, 16);
    TEST_ASSERT_FALSE(Security::validateSecrets(s, true, problems, sizeof(problems)));

    s = validSecrets();
    std::memset(s.displayMac, 0, 6);
    TEST_ASSERT_FALSE(Security::validateSecrets(s, true, problems, sizeof(problems)));
    TEST_ASSERT_TRUE(Security::validateSecrets(s, false, problems, sizeof(problems))); // display needs sender MAC only
}

void test_compiled_secrets_win_and_are_stored() {
    DeviceSecrets compiled = validSecrets();
    DeviceSecrets stored = validSecrets();
    std::strcpy(stored.webPassword, "older-password-1");
    DeviceSecrets out{};
    bool store = false;
    TEST_ASSERT_EQUAL(static_cast<int>(Security::SecretsSource::Compiled),
                      static_cast<int>(Security::chooseSecrets(&compiled, &stored, true, out, store)));
    TEST_ASSERT_TRUE(store);
    TEST_ASSERT_EQUAL_STRING("web-pass-1234", out.webPassword);

    // Identical NVS content is not rewritten (saves flash cycles).
    TEST_ASSERT_EQUAL(static_cast<int>(Security::SecretsSource::Compiled),
                      static_cast<int>(Security::chooseSecrets(&compiled, &compiled, true, out, store)));
    TEST_ASSERT_FALSE(store);
}

void test_release_image_keeps_stored_secrets() {
    // Release builds have no compiled secrets: the NVS copy must be used.
    DeviceSecrets stored = validSecrets();
    DeviceSecrets out{};
    bool store = true;
    TEST_ASSERT_EQUAL(static_cast<int>(Security::SecretsSource::Stored),
                      static_cast<int>(Security::chooseSecrets(nullptr, &stored, true, out, store)));
    TEST_ASSERT_FALSE(store);

    // Placeholder compiled secrets never override valid stored ones.
    DeviceSecrets placeholder = validSecrets();
    std::strcpy(placeholder.webPassword, "change-me-web");
    TEST_ASSERT_EQUAL(static_cast<int>(Security::SecretsSource::Stored),
                      static_cast<int>(Security::chooseSecrets(&placeholder, &stored, true, out, store)));
}

void test_unprovisioned_device_has_no_credentials() {
    DeviceSecrets out{};
    bool store = false;
    TEST_ASSERT_EQUAL(static_cast<int>(Security::SecretsSource::None),
                      static_cast<int>(Security::chooseSecrets(nullptr, nullptr, true, out, store)));
    // Native build ignores local secrets.h, so the runtime is unprovisioned:
    Security::begin(true);
    TEST_ASSERT_FALSE(Security::provisioned());
    TEST_ASSERT_FALSE(WebSecurity::isConfiguredToken("0123456789abcdef0123"));
}

void test_metadata_scanner_handles_split_chunks_and_decoys() {
    const std::string image = std::string("\xE9junk CANOBD2_FW_METADATA_BEGIN;\0decoy", 37) +
                              "CANOBD2_FW_METADATA_BEGIN;target=sender;version=V2.1.0;schema=2;protocol=3;"
                              "min_protocol=3;max_protocol=3;CANOBD2_FW_METADATA_END\0tail";
    for (std::size_t chunk : {std::size_t(1), std::size_t(5), std::size_t(64), image.size()}) {
        FirmwareUpdate::FirmwareMetadataScanner scanner;
        scanner.reset();
        for (std::size_t pos = 0; pos < image.size(); pos += chunk) {
            const std::size_t len = std::min(chunk, image.size() - pos);
            scanner.update(reinterpret_cast<const uint8_t*>(image.data() + pos), len);
        }
        TEST_ASSERT_TRUE(scanner.record().found);
        TEST_ASSERT_EQUAL_STRING("sender", scanner.record().target.c_str());
        TEST_ASSERT_EQUAL_STRING("V2.1.0", scanner.record().version.c_str());
        TEST_ASSERT_EQUAL(3, scanner.record().protocol);
        TEST_ASSERT_EQUAL(2, scanner.record().schema);
    }
}

void test_unsigned_image_is_rejected_by_validator() {
    const char* image = "\xE9 CANOBD2_FW_METADATA_BEGIN;target=sender;version=V2.1.0;schema=2;protocol=3;CANOBD2_FW_METADATA_END";
    FirmwareUpdate::OtaImageValidator validator;
    validator.begin();
    validator.update(reinterpret_cast<const uint8_t*>(image), std::strlen(image));
    const FirmwareUpdate::OtaImageValidator::Expectation expected{"sender", 3, 2};
    TEST_ASSERT_EQUAL(static_cast<int>(FirmwareUpdate::ImageCheck::SignatureInvalid),
                      static_cast<int>(validator.finish(expected, nullptr, 0)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_constant_time_equals);
    RUN_TEST(test_valid_secrets_pass_for_both_targets);
    RUN_TEST(test_placeholders_and_weak_values_are_rejected);
    RUN_TEST(test_compiled_secrets_win_and_are_stored);
    RUN_TEST(test_release_image_keeps_stored_secrets);
    RUN_TEST(test_unprovisioned_device_has_no_credentials);
    RUN_TEST(test_metadata_scanner_handles_split_chunks_and_decoys);
    RUN_TEST(test_unsigned_image_is_rejected_by_validator);
    return UNITY_END();
}
