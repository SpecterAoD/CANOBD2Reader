#include <unity.h>
#include "Crc16.h"

void setUp() {}
void tearDown() {}

void test_crc_modbus_known_vector() {
    const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
    TEST_ASSERT_EQUAL_HEX16(0x4B37, Crc16::modbus(data, sizeof(data)));
}

void test_crc_detects_change() {
    const uint8_t a[] = {0x01, 0x02, 0x03};
    const uint8_t b[] = {0x01, 0x02, 0x04};
    TEST_ASSERT_NOT_EQUAL(Crc16::modbus(a, sizeof(a)), Crc16::modbus(b, sizeof(b)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_crc_modbus_known_vector);
    RUN_TEST(test_crc_detects_change);
    return UNITY_END();
}
