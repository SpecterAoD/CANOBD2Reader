#pragma once

#include <cstddef>
#include <cstdint>

namespace Capabilities {

struct ObdPidDescriptor {
    uint8_t pid;
    const char* name;
    const char* unit;
    bool recommended;
};

constexpr uint8_t SupportedPidRangeCount = 4;

uint8_t supportedRangePidForIndex(uint8_t index);
uint8_t supportedRangeBaseForPid(uint8_t pid);
bool isPidInSupportedRange(uint8_t pid);
bool isPidSupportedByMask(uint8_t pid, uint32_t mask01_20, uint32_t mask21_40, uint32_t mask41_60, uint32_t mask61_80 = 0);
bool maskAdvertisesNextRange(uint8_t rangePid, uint32_t mask);
const ObdPidDescriptor* findObdPidDescriptor(uint8_t pid);
const ObdPidDescriptor* recommendedObdPids(std::size_t& count);

} // namespace Capabilities
