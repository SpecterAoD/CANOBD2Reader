#pragma once

#include <cstdint>

namespace StatusLogic {

enum class Health : uint8_t {
    Ok,
    Warning,
    Error,
    Unknown
};

inline bool senderShouldRun(bool requireWebStart, bool webStartRequested) {
    return !requireWebStart || webStartRequested;
}

inline bool isHeartbeatDue(uint32_t nowMs, uint32_t lastHeartbeatMs, uint32_t intervalMs) {
    return lastHeartbeatMs == 0 || nowMs - lastHeartbeatMs >= intervalMs;
}

inline Health packetLinkHealth(uint32_t nowMs, uint32_t lastPacketMs, uint32_t timeoutMs) {
    if (lastPacketMs == 0) return Health::Error;
    return (nowMs - lastPacketMs <= timeoutMs) ? Health::Ok : Health::Error;
}

inline Health canHealth(bool driverReady, uint32_t nowMs, uint32_t lastCanStatusMs, uint32_t timeoutMs) {
    if (!driverReady) return Health::Error;
    if (lastCanStatusMs == 0) return Health::Warning;
    return (nowMs - lastCanStatusMs <= timeoutMs) ? Health::Ok : Health::Warning;
}

inline Health obdHealth(bool obdEnabled, uint32_t nowMs, uint32_t lastObdResponseMs, uint32_t timeoutMs) {
    if (!obdEnabled) return Health::Unknown;
    if (lastObdResponseMs == 0) return Health::Warning;
    return (nowMs - lastObdResponseMs <= timeoutMs) ? Health::Ok : Health::Warning;
}

inline uint32_t packetLossFromSequence(uint32_t previousSequence, uint32_t nextSequence) {
    if (previousSequence == 0 || nextSequence <= previousSequence + 1) return 0;
    return nextSequence - previousSequence - 1;
}

}
