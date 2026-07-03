#include "CanSignalDiff.h"

namespace Capabilities {

std::size_t diffCanFrames(const CanFrameSample& before,
                          const CanFrameSample& after,
                          CanSignalCandidate* out,
                          std::size_t outCount) {
    if (out == nullptr || outCount == 0) return 0;
    if (before.canId != after.canId) return 0;

    const uint8_t length = before.length < after.length ? before.length : after.length;
    std::size_t written = 0;
    for (uint8_t index = 0; index < length && written < outCount && index < 8; ++index) {
        const uint8_t changed = static_cast<uint8_t>(before.data[index] ^ after.data[index]);
        if (changed == 0) continue;

        out[written].canId = before.canId;
        out[written].byteIndex = index;
        out[written].beforeValue = before.data[index];
        out[written].afterValue = after.data[index];
        out[written].changedBitMask = changed;
        out[written].changeCount = 1;
        ++written;
    }
    return written;
}

} // namespace Capabilities
