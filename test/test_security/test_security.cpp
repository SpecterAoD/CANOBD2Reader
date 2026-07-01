#include <unity.h>
#include "AuthHelpers.h"
#include "SecurityConfig.h"

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

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_constant_time_equals);
    RUN_TEST(test_authentication_config_is_present);
    return UNITY_END();
}
