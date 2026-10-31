#include <unity.h>

#include "UpdateChannel.h"
#include "config/UpdateConfig.h"

void setUp() {}
void tearDown() {}

// Devices only offer stable releases unless the user picks another channel.
void test_default_channel_is_stable() {
    TEST_ASSERT_EQUAL(FirmwareUpdate::UpdateChannel::Stable, UpdateConfig::DefaultChannel);
}

void test_channel_filtering() {
    TEST_ASSERT_TRUE(FirmwareUpdate::channelAllows(FirmwareUpdate::UpdateChannel::Stable,
                                                   FirmwareUpdate::UpdateChannel::Stable));
    TEST_ASSERT_FALSE(FirmwareUpdate::channelAllows(FirmwareUpdate::UpdateChannel::Stable,
                                                    FirmwareUpdate::UpdateChannel::Beta));
    TEST_ASSERT_TRUE(FirmwareUpdate::channelAllows(FirmwareUpdate::UpdateChannel::Beta,
                                                   FirmwareUpdate::UpdateChannel::Stable));
    TEST_ASSERT_TRUE(FirmwareUpdate::channelAllows(FirmwareUpdate::UpdateChannel::Beta,
                                                   FirmwareUpdate::UpdateChannel::Beta));
    TEST_ASSERT_TRUE(FirmwareUpdate::channelAllows(FirmwareUpdate::UpdateChannel::Development,
                                                   FirmwareUpdate::UpdateChannel::Stable));
    TEST_ASSERT_TRUE(FirmwareUpdate::channelAllows(FirmwareUpdate::UpdateChannel::Development,
                                                   FirmwareUpdate::UpdateChannel::Development));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_default_channel_is_stable);
    RUN_TEST(test_channel_filtering);
    return UNITY_END();
}
