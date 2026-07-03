#include "CapabilityTypes.h"

namespace Capabilities {

const char* statusText(CapabilityStatus status) {
    switch (status) {
        case CapabilityStatus::Unknown: return "UNKNOWN";
        case CapabilityStatus::Ok: return "OK";
        case CapabilityStatus::Unsupported: return "UNSUPPORTED";
        case CapabilityStatus::Timeout: return "TIMEOUT";
        case CapabilityStatus::DecodeError: return "DECODE_ERROR";
        case CapabilityStatus::Pending: return "PENDING";
        case CapabilityStatus::NegativeResponse: return "NEGATIVE_RESPONSE";
        case CapabilityStatus::SendFailed: return "SEND_FAILED";
    }
    return "UNKNOWN";
}

bool isTerminal(CapabilityStatus status) {
    return status != CapabilityStatus::Unknown && status != CapabilityStatus::Pending;
}

} // namespace Capabilities
