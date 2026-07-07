#include "UdsCapability.h"
#include "config/UdsConfig.h"

namespace Capabilities {
namespace {

constexpr UdsServiceProbe kServiceProbes[] = {
    {0x10, "DiagnosticSessionControl"},
    {0x22, "ReadDataByIdentifier"},
    {0x19, "ReadDTCInformation"},
    {0x3E, "TesterPresent"},
};

} // namespace

// NRC 0x78 ("ResponsePending") is a soft hold signal from the ECU, not a failure.
// The ECU sends it while still computing a response and expects the tester to keep
// the session alive and continue waiting. Treating 0x78 as an error would abort
// diagnostics prematurely on slow or heavily loaded ECUs.
bool isUdsResponsePending(uint8_t nrc) {
    return nrc == 0x78;
}

bool isUdsUnsupportedNrc(uint8_t nrc) {
    return nrc == 0x11 || nrc == 0x12 || nrc == 0x31;
}

CapabilityStatus statusForNegativeResponse(uint8_t nrc) {
    // Pending maps to its own status so the scan loop can decide whether to
    // keep waiting rather than treating the service as definitively broken.
    if (isUdsResponsePending(nrc)) return CapabilityStatus::Pending;
    // NRCs 0x11, 0x12 and 0x31 indicate the service or sub-function is simply
    // not offered by this ECU; mark it Unsupported rather than as an error.
    if (isUdsUnsupportedNrc(nrc)) return CapabilityStatus::Unsupported;
    return CapabilityStatus::NegativeResponse;
}

// While the ECU continues sending 0x78 ResponsePending, we keep polling until
// totalTimeoutMs elapses. The hard upper bound prevents indefinite waits if a
// misbehaving ECU never delivers a final positive or negative response.
bool shouldContinueWaitingForPending(uint32_t startMs, uint32_t nowMs, uint32_t totalTimeoutMs) {
    return nowMs - startMs < totalTimeoutMs;
}

uint32_t udsResponseIdForRequestId(uint32_t requestId) {
    return UdsConfig::responseIdForRequestId(requestId);
}

bool isUdsRequestIdInScanRange(uint32_t requestId) {
    return (requestId >= UdsFirstRequestId && requestId <= UdsLastRequestId) ||
           UdsConfig::isConfiguredRequestId(requestId);
}

const UdsServiceProbe* udsServiceProbes(uint8_t& count) {
    count = sizeof(kServiceProbes) / sizeof(kServiceProbes[0]);
    return kServiceProbes;
}

const UdsDidProbe* udsDidProbes(uint8_t& count) {
    static UdsDidProbe probes[UdsConfig::DidCandidateCount]{};
    for (std::size_t index = 0; index < UdsConfig::DidCandidateCount; ++index) {
        probes[index] = {
            UdsConfig::DidCandidates[index].did,
            UdsConfig::DidCandidates[index].name,
            UdsConfig::DidCandidates[index].scanByDefault
        };
    }
    count = static_cast<uint8_t>(UdsConfig::DidCandidateCount);
    return probes;
}

} // namespace Capabilities
