#pragma once

#include <cstddef>
#include <cstdint>

namespace Uds {

bool decodeAsciiDid(const uint8_t* payload,
                    std::size_t length,
                    uint16_t expectedDid,
                    char* out,
                    std::size_t outSize);

bool isPositiveDidResponse(const uint8_t* payload, std::size_t length, uint16_t expectedDid);
bool isPositiveDtcResponse(const uint8_t* payload, std::size_t length);

} // namespace Uds
