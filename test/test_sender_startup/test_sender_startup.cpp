#include <unity.h>
#include "SenderConfig.h"
#include "StatusLogic.h"

void setUp() {}
void tearDown() {}

void test_sender_auto_start_is_default() {
    TEST_ASSERT_FALSE(SenderConfig::RequireWebStart);
    TEST_ASSERT_TRUE(StatusLogic::senderShouldRun(SenderConfig::RequireWebStart, false));
}

void test_manual_start_mode_still_supported() {
    TEST_ASSERT_FALSE(StatusLogic::senderShouldRun(true, false));
    TEST_ASSERT_TRUE(StatusLogic::senderShouldRun(true, true));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_sender_auto_start_is_default);
    RUN_TEST(test_manual_start_mode_still_supported);
    return UNITY_END();
}
