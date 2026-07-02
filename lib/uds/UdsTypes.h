#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace Uds {

enum class Service : uint8_t {
    DiagnosticSessionControl = 0x10,
    ECUReset = 0x11,
    ReadDTCInformation = 0x19,
    ReadDataByIdentifier = 0x22,
    SecurityAccess = 0x27,
    TesterPresent = 0x3E
};

enum class NegativeResponseCode : uint8_t {
    GeneralReject = 0x10,
    ServiceNotSupported = 0x11,
    SubFunctionNotSupported = 0x12,
    IncorrectMessageLength = 0x13,
    BusyRepeatRequest = 0x21,
    ConditionsNotCorrect = 0x22,
    RequestOutOfRange = 0x31,
    SecurityAccessDenied = 0x33,
    ResponsePending = 0x78
};

struct Response {
    bool positive = false;
    bool negative = false;
    Service service = Service::TesterPresent;
    uint8_t negativeService = 0;
    uint8_t negativeCode = 0;
    uint32_t responseId = 0;
    const uint8_t* data = nullptr;
    std::size_t length = 0;
};

inline uint8_t serviceId(Service service) {
    return static_cast<uint8_t>(service);
}

inline uint8_t positiveResponseId(Service service) {
    return static_cast<uint8_t>(serviceId(service) + 0x40U);
}

inline const char* serviceName(Service service) {
    switch (service) {
        case Service::DiagnosticSessionControl: return "DiagnosticSessionControl";
        case Service::ECUReset: return "ECUReset";
        case Service::ReadDTCInformation: return "ReadDTCInformation";
        case Service::ReadDataByIdentifier: return "ReadDataByIdentifier";
        case Service::SecurityAccess: return "SecurityAccess";
        case Service::TesterPresent: return "TesterPresent";
    }
    return "UnknownService";
}

inline const char* negativeResponseDescription(uint8_t nrc) {
    switch (nrc) {
        case 0x10: return "GeneralReject";
        case 0x11: return "ServiceNotSupported";
        case 0x12: return "SubFunctionNotSupported";
        case 0x13: return "IncorrectMessageLength";
        case 0x21: return "BusyRepeatRequest";
        case 0x22: return "ConditionsNotCorrect";
        case 0x31: return "RequestOutOfRange";
        case 0x33: return "SecurityAccessDenied";
        case 0x78: return "ResponsePending";
        default: return "UnknownNRC";
    }
}

inline void formatDid(uint16_t did, char* out, std::size_t outSize) {
    if (out == nullptr || outSize == 0) return;
    std::snprintf(out, outSize, "0x%04X", static_cast<unsigned>(did));
}

} // namespace Uds
