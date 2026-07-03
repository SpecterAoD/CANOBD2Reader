#include "ObdCapability.h"

namespace Capabilities {
namespace {

constexpr ObdPidDescriptor kRecommendedPids[] = {
    {0x04, "EngineLoad", "%", true},
    {0x05, "CoolantTemp", "C", true},
    {0x0B, "MAP", "kPa", true},
    {0x0C, "RPM", "rpm", true},
    {0x0D, "Speed", "km/h", true},
    {0x0F, "IntakeTemp", "C", true},
    {0x10, "MAF", "g/s", true},
    {0x11, "Throttle", "%", true},
    {0x1F, "RunTime", "s", true},
    {0x2F, "FuelLevel", "%", true},
    {0x33, "BarometricPressure", "kPa", true},
    {0x42, "ControlVoltage", "V", true},
    {0x46, "AmbientTemp", "C", true},
    {0x5C, "OilTemp", "C", true},
    {0x5E, "FuelRate", "L/h", true},
};

bool bitInMask(uint8_t pid, uint8_t basePid, uint32_t mask) {
    if (pid <= basePid || pid > static_cast<uint8_t>(basePid + 32U)) return false;
    const uint8_t offset = static_cast<uint8_t>(pid - basePid);
    return (mask & (1UL << (32U - offset))) != 0;
}

} // namespace

uint8_t supportedRangePidForIndex(uint8_t index) {
    switch (index) {
        case 0: return 0x00;
        case 1: return 0x20;
        case 2: return 0x40;
        case 3: return 0x60;
        default: return 0x00;
    }
}

uint8_t supportedRangeBaseForPid(uint8_t pid) {
    if (pid >= 0x01 && pid <= 0x20) return 0x00;
    if (pid >= 0x21 && pid <= 0x40) return 0x20;
    if (pid >= 0x41 && pid <= 0x60) return 0x40;
    if (pid >= 0x61 && pid <= 0x80) return 0x60;
    return 0xFF;
}

bool isPidInSupportedRange(uint8_t pid) {
    return supportedRangeBaseForPid(pid) != 0xFF;
}

bool isPidSupportedByMask(uint8_t pid,
                          uint32_t mask01_20,
                          uint32_t mask21_40,
                          uint32_t mask41_60,
                          uint32_t mask61_80) {
    switch (supportedRangeBaseForPid(pid)) {
        case 0x00: return bitInMask(pid, 0x00, mask01_20);
        case 0x20: return bitInMask(pid, 0x20, mask21_40);
        case 0x40: return bitInMask(pid, 0x40, mask41_60);
        case 0x60: return bitInMask(pid, 0x60, mask61_80);
        default: return false;
    }
}

bool maskAdvertisesNextRange(uint8_t rangePid, uint32_t mask) {
    return rangePid <= 0x60 && (mask & 0x00000001UL) != 0;
}

const ObdPidDescriptor* findObdPidDescriptor(uint8_t pid) {
    for (const auto& descriptor : kRecommendedPids) {
        if (descriptor.pid == pid) return &descriptor;
    }
    return nullptr;
}

const ObdPidDescriptor* recommendedObdPids(std::size_t& count) {
    count = sizeof(kRecommendedPids) / sizeof(kRecommendedPids[0]);
    return kRecommendedPids;
}

} // namespace Capabilities
