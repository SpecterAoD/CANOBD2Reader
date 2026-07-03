#include "UdsCapability.h"

namespace Capabilities {
namespace {

constexpr UdsServiceProbe kServiceProbes[] = {
    {0x10, "DiagnosticSessionControl"},
    {0x22, "ReadDataByIdentifier"},
    {0x19, "ReadDTCInformation"},
    {0x3E, "TesterPresent"},
};

constexpr UdsDidProbe kDidProbes[] = {
    {0xF180, "BootSoftwareIdentification"},
    {0xF181, "ApplicationSoftwareIdentification"},
    {0xF182, "ApplicationDataIdentification"},
    {0xF187, "ManufacturerSparePartNumber"},
    {0xF188, "EcuSoftwareNumber"},
    {0xF189, "EcuSoftwareVersion"},
    {0xF18A, "SystemSupplierIdentifier"},
    {0xF18B, "EcuManufacturingDate"},
    {0xF18C, "EcuSerialNumber"},
    {0xF190, "VIN"},
};

} // namespace

bool isUdsResponsePending(uint8_t nrc) {
    return nrc == 0x78;
}

bool isUdsUnsupportedNrc(uint8_t nrc) {
    return nrc == 0x11 || nrc == 0x12 || nrc == 0x31;
}

CapabilityStatus statusForNegativeResponse(uint8_t nrc) {
    if (isUdsResponsePending(nrc)) return CapabilityStatus::Pending;
    if (isUdsUnsupportedNrc(nrc)) return CapabilityStatus::Unsupported;
    return CapabilityStatus::NegativeResponse;
}

bool shouldContinueWaitingForPending(uint32_t startMs, uint32_t nowMs, uint32_t totalTimeoutMs) {
    return nowMs - startMs < totalTimeoutMs;
}

uint32_t udsResponseIdForRequestId(uint32_t requestId) {
    return requestId + 8U;
}

bool isUdsRequestIdInScanRange(uint32_t requestId) {
    return requestId >= UdsFirstRequestId && requestId <= UdsLastRequestId;
}

const UdsServiceProbe* udsServiceProbes(uint8_t& count) {
    count = sizeof(kServiceProbes) / sizeof(kServiceProbes[0]);
    return kServiceProbes;
}

const UdsDidProbe* udsDidProbes(uint8_t& count) {
    count = sizeof(kDidProbes) / sizeof(kDidProbes[0]);
    return kDidProbes;
}

} // namespace Capabilities
