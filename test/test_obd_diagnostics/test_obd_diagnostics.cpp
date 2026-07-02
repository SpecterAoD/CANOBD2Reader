#include <unity.h>
#include <cstring>

#include "DtcDecoder.h"
#include "ObdDiagnostics.h"
#include "VinDecoder.h"

void setUp() {}
void tearDown() {}

void test_obd_diagnostics_counts_and_fallback() {
    Obd::Diagnostics::reset();
    TEST_ASSERT_EQUAL_UINT32(0, Obd::Diagnostics::requestCount());
    TEST_ASSERT_FALSE(Obd::Diagnostics::physicalFallbackActive());

    Obd::Diagnostics::recordRequest(0x01, 0x0C, IsoTp::FunctionalRequestId);
    Obd::Diagnostics::recordTimeout();
    Obd::Diagnostics::recordTimeout();
    Obd::Diagnostics::recordTimeout();

    TEST_ASSERT_EQUAL_UINT32(1, Obd::Diagnostics::requestCount());
    TEST_ASSERT_EQUAL_UINT32(3, Obd::Diagnostics::timeoutCount());
    TEST_ASSERT_TRUE(Obd::Diagnostics::shouldSwitchToPhysicalFallback(3));

    Obd::Diagnostics::setPhysicalFallbackActive(true);
    TEST_ASSERT_TRUE(Obd::Diagnostics::physicalFallbackActive());
    TEST_ASSERT_EQUAL_UINT32(IsoTp::PhysicalRequestId, Obd::Diagnostics::requestCanId());
}

void test_negative_response_resets_timeout_streak() {
    Obd::Diagnostics::reset();
    Obd::Diagnostics::recordTimeout();
    Obd::Diagnostics::recordNegativeResponse(0x01, 0x11);

    TEST_ASSERT_EQUAL_UINT32(1, Obd::Diagnostics::negativeResponseCount());
    TEST_ASSERT_EQUAL_UINT32(0, Obd::Diagnostics::timeoutStreak());
    TEST_ASSERT_NOT_NULL(std::strstr(Obd::Diagnostics::lastNegativeResponse(), "ServiceNotSupported"));
}

void test_dtc_decoder() {
    const uint8_t dtcBytes[] = {0x01, 0x33, 0x04, 0x20, 0x00, 0x00};
    char text[32] = {};
    const std::size_t count = Obd::decodeDtcList(dtcBytes, sizeof(dtcBytes), text, sizeof(text));

    TEST_ASSERT_EQUAL_UINT32(2, count);
    TEST_ASSERT_EQUAL_STRING("P0133 P0420", text);
}

void test_vin_decoder() {
    const uint8_t vinPayload[] = {
        0x49, 0x02, 0x01,
        'W', 'V', 'W', 'Z', 'Z', 'Z', '1', 'J', 'Z', 'X', 'W', '0', '0', '0', '0', '0', '1'
    };
    char vin[24] = {};

    TEST_ASSERT_TRUE(Obd::decodeVin(vinPayload, sizeof(vinPayload), vin, sizeof(vin)));
    TEST_ASSERT_EQUAL_STRING("WVWZZZ1JZXW000001", vin);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_obd_diagnostics_counts_and_fallback);
    RUN_TEST(test_negative_response_resets_timeout_streak);
    RUN_TEST(test_dtc_decoder);
    RUN_TEST(test_vin_decoder);
    return UNITY_END();
}
