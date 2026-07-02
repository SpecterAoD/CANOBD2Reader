#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace Obd {

inline char dtcTypeChar(uint8_t highBits) {
    switch (highBits & 0x03U) {
        case 0: return 'P';
        case 1: return 'C';
        case 2: return 'B';
        default: return 'U';
    }
}

/// Decodes Mode 03 DTC bytes into a space-separated list such as "P0133 P0420".
/// The input must start at the first DTC byte, i.e. after the positive-response
/// service byte 0x43. Zero-padding pairs are ignored.
inline std::size_t decodeDtcList(const uint8_t* data, std::size_t length, char* out, std::size_t outSize) {
    if (out == nullptr || outSize == 0) return 0;
    out[0] = '\0';
    if (data == nullptr || length < 2) return 0;

    std::size_t count = 0;
    for (std::size_t index = 0; index + 1 < length; index += 2) {
        const uint8_t a = data[index];
        const uint8_t b = data[index + 1];
        if (a == 0 && b == 0) continue;

        char code[8];
        std::snprintf(code, sizeof(code), "%c%01X%03X",
                      dtcTypeChar(a >> 6U),
                      (a >> 4U) & 0x03U,
                      ((a & 0x0FU) << 8U) | b);

        const std::size_t used = std::strlen(out);
        if (used >= outSize - 1) break;
        std::snprintf(out + used, outSize - used, "%s%s", used > 0 ? " " : "", code);
        ++count;
    }
    return count;
}

} // namespace Obd
