#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace Obd {

/// Extracts the VIN from a Mode 09 PID 02 positive ISO-TP payload.
///
/// Typical payload:
///   49 02 01 57 56 57 ...
/// The third byte is the VIN message count/index and is skipped when present.
inline bool decodeVin(const uint8_t* payload, std::size_t length, char* out, std::size_t outSize) {
    if (out == nullptr || outSize == 0) return false;
    out[0] = '\0';
    if (payload == nullptr || length < 3) return false;
    if (payload[0] != 0x49 || payload[1] != 0x02) return false;

    const std::size_t start = length >= 20 ? 3 : 2;
    std::size_t written = 0;
    for (std::size_t index = start; index < length && written < 17 && written + 1 < outSize; ++index) {
        const uint8_t c = payload[index];
        if (c < 0x21 || c > 0x7E) continue;
        out[written++] = static_cast<char>(c);
    }
    out[written] = '\0';
    return written > 0;
}

} // namespace Obd
