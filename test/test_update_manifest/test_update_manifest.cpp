#include <unity.h>

#include "UpdateManifest.h"

void setUp() {}
void tearDown() {}

namespace {
const char* kManifest = R"json(
{
  "manifestVersion": 1,
  "project": "CANOBD2Reader",
  "sender": [
    {"version":"V2.0.0.b5","channel":"development","url":"https://example/sender-b5.bin","sha256":"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa","protocol":2,"createdAt":"2026-07-06T00:00:00Z"},
    {"version":"V2.0.0","channel":"stable","url":"https://example/sender-200.bin","sha256":"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb","protocol":2,"createdAt":"2026-07-01T00:00:00Z"},
    {"version":"V1.0.13","channel":"stable","url":"https://example/sender-1013.bin","sha256":"cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc","protocol":2,"createdAt":"2026-06-01T00:00:00Z"},
    {"version":"V3.0.0","channel":"development","url":"https://example/sender-300.bin","sha256":"dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd","protocol":3,"createdAt":"2026-08-01T00:00:00Z"}
  ],
  "display": [
    {"version":"V2.0.0.b5","channel":"development","url":"https://example/display-b5.bin","sha256":"eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee","protocol":2,"createdAt":"2026-07-06T00:00:00Z"}
  ]
}
)json";
}

void test_manifest_parse_and_best_update() {
    FirmwareUpdate::UpdateManifest manifest;
    TEST_ASSERT_TRUE(FirmwareUpdate::parseManifest(kManifest, manifest));
    TEST_ASSERT_EQUAL(4, manifest.sender.size());
    const auto* best = FirmwareUpdate::findBestForwardUpdate(manifest,
                                                             "sender",
                                                             2,
                                                             "V2.0.0.b4",
                                                             FirmwareUpdate::UpdateChannel::Development);
    TEST_ASSERT_NOT_NULL(best);
    TEST_ASSERT_EQUAL_STRING("V2.0.0.b5", best->version.c_str());
}

void test_stable_filters_development() {
    FirmwareUpdate::UpdateManifest manifest;
    TEST_ASSERT_TRUE(FirmwareUpdate::parseManifest(kManifest, manifest));
    const auto* best = FirmwareUpdate::findBestForwardUpdate(manifest,
                                                             "sender",
                                                             2,
                                                             "V1.0.13",
                                                             FirmwareUpdate::UpdateChannel::Stable);
    TEST_ASSERT_NOT_NULL(best);
    TEST_ASSERT_EQUAL_STRING("V2.0.0", best->version.c_str());
}

void test_wrong_target_protocol_and_sha_are_rejected() {
    FirmwareUpdate::FirmwareVersionEntry entry;
    entry.version = "V2.0.0.b5";
    entry.channel = FirmwareUpdate::UpdateChannel::Development;
    entry.target = "display";
    entry.protocol = 2;
    entry.url = "https://example/display.bin";
    entry.sha256 = "eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee";
    TEST_ASSERT_EQUAL(FirmwareUpdate::InstallEligibility::WrongTarget,
                      FirmwareUpdate::evaluateEntry(entry, "sender", 2, "V2.0.0.b4",
                                                    FirmwareUpdate::UpdateChannel::Development, true));
    entry.target = "sender";
    entry.protocol = 3;
    TEST_ASSERT_EQUAL(FirmwareUpdate::InstallEligibility::IncompatibleProtocol,
                      FirmwareUpdate::evaluateEntry(entry, "sender", 2, "V2.0.0.b4",
                                                    FirmwareUpdate::UpdateChannel::Development, true));
    entry.protocol = 2;
    entry.sha256.clear();
    TEST_ASSERT_EQUAL(FirmwareUpdate::InstallEligibility::MissingSha256,
                      FirmwareUpdate::evaluateEntry(entry, "sender", 2, "V2.0.0.b4",
                                                    FirmwareUpdate::UpdateChannel::Development, true));
}

void test_rollback_is_manual_only_policy() {
    FirmwareUpdate::FirmwareVersionEntry entry;
    entry.version = "V1.0.13";
    entry.channel = FirmwareUpdate::UpdateChannel::Stable;
    entry.target = "sender";
    entry.protocol = 2;
    entry.url = "https://example/sender.bin";
    entry.sha256 = "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc";
    TEST_ASSERT_EQUAL(FirmwareUpdate::InstallEligibility::Rollback,
                      FirmwareUpdate::evaluateEntry(entry, "sender", 2, "V2.0.0.b4",
                                                    FirmwareUpdate::UpdateChannel::Development, true));
    TEST_ASSERT_EQUAL(FirmwareUpdate::InstallEligibility::Installed,
                      FirmwareUpdate::evaluateEntry(entry, "sender", 2, "V2.0.0.b4",
                                                    FirmwareUpdate::UpdateChannel::Development, false));
}

void test_find_version_accepts_optional_v_prefix() {
    FirmwareUpdate::UpdateManifest manifest;
    TEST_ASSERT_TRUE(FirmwareUpdate::parseManifest(kManifest, manifest));
    const auto* withPrefix = FirmwareUpdate::findVersion(manifest, "sender", "V1.0.13");
    const auto* withoutPrefix = FirmwareUpdate::findVersion(manifest, "sender", "1.0.13");
    TEST_ASSERT_NOT_NULL(withPrefix);
    TEST_ASSERT_NOT_NULL(withoutPrefix);
    TEST_ASSERT_EQUAL_STRING(withPrefix->version.c_str(), withoutPrefix->version.c_str());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_manifest_parse_and_best_update);
    RUN_TEST(test_stable_filters_development);
    RUN_TEST(test_wrong_target_protocol_and_sha_are_rejected);
    RUN_TEST(test_rollback_is_manual_only_policy);
    RUN_TEST(test_find_version_accepts_optional_v_prefix);
    return UNITY_END();
}
