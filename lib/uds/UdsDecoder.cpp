#include "UdsDecoder.h"

namespace Uds {

bool isPositiveDidResponse(const uint8_t* payload, std::size_t length, uint16_t expectedDid) {
    if (payload == nullptr || length < 3) return false;
    if (payload[0] != 0x62) return false;
    const uint16_t did = (static_cast<uint16_t>(payload[1]) << 8U) | payload[2];
    return did == expectedDid;
}

bool decodeAsciiDid(const uint8_t* payload,
                    std::size_t length,
                    uint16_t expectedDid,
                    char* out,
                    std::size_t outSize) {
    if (out == nullptr || outSize == 0) return false;
    out[0] = '\0';
    if (!isPositiveDidResponse(payload, length, expectedDid)) return false;

    std::size_t written = 0;
    for (std::size_t index = 3; index < length && written + 1 < outSize; ++index) {
        const uint8_t c = payload[index];
        if (c < 0x20 || c > 0x7E) continue;
        out[written++] = static_cast<char>(c);
    }
    out[written] = '\0';
    return written > 0;
}

bool isPositiveDtcResponse(const uint8_t* payload, std::size_t length) {
    return payload != nullptr && length >= 2 && payload[0] == 0x59;
}

} // namespace Uds
