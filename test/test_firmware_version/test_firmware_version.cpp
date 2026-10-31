// SemVer ordering of firmware versions, including the project's legacy tags.
#include <unity.h>

#include "FirmwareVersion.h"

using FirmwareUpdate::FirmwareVersion;
using FirmwareUpdate::compareVersions;

void setUp() {}
void tearDown() {}

void test_final_release_is_newer_than_its_prereleases() {
    TEST_ASSERT_TRUE(compareVersions("V2.0.0", "V2.0.0-beta.6") > 0);
    TEST_ASSERT_TRUE(compareVersions("V2.0.0-rc.1", "V2.0.0") < 0);
}

void test_prerelease_numbers_compare_numerically() {
    TEST_ASSERT_TRUE(compareVersions("2.0.0-beta.10", "2.0.0-beta.9") > 0);
    TEST_ASSERT_TRUE(compareVersions("2.0.0-beta.2", "2.0.0-beta.10") < 0);
}

void test_prerelease_kinds_follow_semver() {
    TEST_ASSERT_TRUE(compareVersions("2.0.0-alpha.1", "2.0.0-beta.1") < 0);
    TEST_ASSERT_TRUE(compareVersions("2.0.0-beta.3", "2.0.0-rc.1") < 0);
    TEST_ASSERT_TRUE(compareVersions("2.0.0-beta", "2.0.0-beta.1") < 0);
}

void test_legacy_tags_match_semver_tags() {
    TEST_ASSERT_EQUAL(0, compareVersions("V2.0.0.b6", "V2.0.0-beta.6"));
    TEST_ASSERT_EQUAL(0, compareVersions("2.0.0-Beta6", "2.0.0-beta.6"));
    TEST_ASSERT_EQUAL_STRING("2.0.0-beta.6", FirmwareVersion::parse("V2.0.0.b6").toString().c_str());
    TEST_ASSERT_EQUAL_STRING("1.0.15-test.1", FirmwareVersion::parse("1.0.15.test1").toString().c_str());
}

void test_core_numbers_compare_numerically() {
    TEST_ASSERT_TRUE(compareVersions("2.0.10", "2.0.9") > 0);
    TEST_ASSERT_TRUE(compareVersions("10.0.0", "9.9.9") > 0);
    TEST_ASSERT_EQUAL(0, compareVersions("v1.2.3", "1.2.3"));
    TEST_ASSERT_EQUAL(0, compareVersions("1.2.3+build.7", "1.2.3"));
}

void test_invalid_versions_never_look_newer() {
    TEST_ASSERT_FALSE(FirmwareVersion::parse("").valid);
    TEST_ASSERT_FALSE(FirmwareVersion::parse("2.0").valid);
    TEST_ASSERT_FALSE(FirmwareVersion::parse("2.0.0-be ta").valid);
    TEST_ASSERT_TRUE(compareVersions("garbage", "0.0.1") < 0);
    TEST_ASSERT_TRUE(compareVersions("0.0.1", "garbage") > 0);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_final_release_is_newer_than_its_prereleases);
    RUN_TEST(test_prerelease_numbers_compare_numerically);
    RUN_TEST(test_prerelease_kinds_follow_semver);
    RUN_TEST(test_legacy_tags_match_semver_tags);
    RUN_TEST(test_core_numbers_compare_numerically);
    RUN_TEST(test_invalid_versions_never_look_newer);
    return UNITY_END();
}
