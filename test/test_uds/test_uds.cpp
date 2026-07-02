#include <unity.h>
#include <cstring>

#include "UdsClient.h"
#include "UdsDecoder.h"
#include "UdsDiagnostics.h"
#include "UdsTypes.h"

void setUp() {
    Uds::Diagnostics::reset();
}

void tearDown() {}

void test_uds_service_allowlist_blocks_unsafe_services() {
    TEST_ASSERT_TRUE(Uds::Client::isServiceAllowed(Uds::Service::ReadDataByIdentifier));
    TEST_ASSERT_TRUE(Uds::Client::isServiceAllowed(Uds::Service::ReadDTCInformation));
    TEST_ASSERT_TRUE(Uds::Client::isServiceAllowed(Uds::Service::TesterPresent));
    TEST_ASSERT_TRUE(Uds::Client::isServiceAllowed(Uds::Service::DiagnosticSessionControl));
    TEST_ASSERT_FALSE(Uds::Client::isServiceAllowed(Uds::Service::ECUReset));
    TEST_ASSERT_FALSE(Uds::Client::isServiceAllowed(Uds::Service::SecurityAccess));
}

void test_uds_ascii_did_decoder() {
    const uint8_t payload[] = {
        0x62, 0xF1, 0x90,
        'W', 'V', 'W', 'Z', 'Z', 'Z', '1', 'J', 'Z', 'X', 'W', '0', '0', '0', '0', '0', '1'
    };
    char vin[24] = {};

    TEST_ASSERT_TRUE(Uds::decodeAsciiDid(payload, sizeof(payload), 0xF190, vin, sizeof(vin)));
    TEST_ASSERT_EQUAL_STRING("WVWZZZ1JZXW000001", vin);
    TEST_ASSERT_FALSE(Uds::decodeAsciiDid(payload, sizeof(payload), 0xF187, vin, sizeof(vin)));
}

void test_uds_diagnostics_records_nrc() {
    const uint8_t request[] = {0x22, 0xF1, 0x90};
    Uds::Diagnostics::recordRequest(Uds::Service::ReadDataByIdentifier, request, sizeof(request), 0x7E0);
    Uds::Diagnostics::recordNegativeResponse(0x22, 0x31);

    TEST_ASSERT_EQUAL_UINT32(1, Uds::Diagnostics::requestCount());
    TEST_ASSERT_EQUAL_UINT32(1, Uds::Diagnostics::negativeResponseCount());
    TEST_ASSERT_NOT_NULL(std::strstr(Uds::Diagnostics::lastNegativeResponse(), "RequestOutOfRange"));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_uds_service_allowlist_blocks_unsafe_services);
    RUN_TEST(test_uds_ascii_did_decoder);
    RUN_TEST(test_uds_diagnostics_records_nrc);
    return UNITY_END();
}
