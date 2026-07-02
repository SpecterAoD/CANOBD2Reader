#include <unity.h>
#include "DiagnosticLog.h"
#include "config/LoggingConfig.h"
#include "config/SenderConfig.h"

void setUp() {}
void tearDown() {}

void test_rotation_helper_uses_configured_limit() {
    TEST_ASSERT_FALSE(DiagnosticLog::shouldRotate(10, 5, SenderConfig::DiagnosticLogMaxBytes));
    TEST_ASSERT_TRUE(DiagnosticLog::shouldRotate(SenderConfig::DiagnosticLogMaxBytes - 1, 2, SenderConfig::DiagnosticLogMaxBytes));
}

void test_diagnostic_log_config_is_valid() {
    TEST_ASSERT_TRUE(SenderConfig::EnablePersistentDiagnosticLog);
    TEST_ASSERT_TRUE(LoggingConfig::EnablePersistentDiagnosticLog);
    TEST_ASSERT_GREATER_THAN(1024U, SenderConfig::DiagnosticLogMaxBytes);
    TEST_ASSERT_EQUAL_UINT(SenderConfig::DiagnosticLogMaxBytes, LoggingConfig::DiagnosticLogMaxBytes);
    TEST_ASSERT_NOT_NULL(SenderConfig::DiagnosticLogPath);
    TEST_ASSERT_NOT_NULL(SenderConfig::DiagnosticLogArchivePath);
    TEST_ASSERT_NOT_NULL(LoggingConfig::DisplayDiagnosticLogPath);
    TEST_ASSERT_NOT_NULL(LoggingConfig::DisplayDiagnosticLogArchivePath);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_rotation_helper_uses_configured_limit);
    RUN_TEST(test_diagnostic_log_config_is_valid);
    return UNITY_END();
}
