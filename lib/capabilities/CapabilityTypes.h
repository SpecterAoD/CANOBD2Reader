#pragma once

#include <cstddef>
#include <cstdint>

namespace Capabilities {

constexpr std::size_t MaxObdPid = 0x80;
constexpr std::size_t MaxObdPidResults = MaxObdPid + 1;
constexpr std::size_t MaxEcuResults = 10;
constexpr std::size_t MaxDidResults = MaxEcuResults * 24;
constexpr std::size_t MaxCanSignalCandidates = 32;

enum class ScanState : uint8_t {
    Idle,
    Running,
    Complete,
    Stopped,
    Error
};

enum class CapabilityStatus : uint8_t {
    Unknown,
    Ok,
    Unsupported,
    Timeout,
    DecodeError,
    Pending,
    NegativeResponse,
    SendFailed
};

struct ObdPidCapability {
    uint8_t pid = 0;
    bool supportedByMask = false;
    bool selectedRuntime = false;
    bool displayRuntime = false;
    bool responded = false;
    bool validDecoded = false;
    uint32_t responseTimeMs = 0;
    char name[32] = {};
    char unit[8] = {};
    char exampleValue[24] = {};
    char status[16] = "UNKNOWN";
};

struct EcuCapability {
    uint32_t requestId = 0;
    uint32_t responseId = 0;
    bool reachable = false;
    bool supportsUds = false;
    bool supportsObd = false;
    char vin[24] = {};
    char ecuName[48] = {};
    char status[20] = "UNKNOWN";
};

struct UdsDidCapability {
    uint32_t requestId = 0;
    uint32_t responseId = 0;
    uint16_t did = 0;
    bool responded = false;
    bool pendingSeen = false;
    uint16_t pendingCount = 0;
    char name[48] = {};
    char value[64] = {};
    char status[20] = "UNKNOWN";
};

struct CanSignalCandidate {
    uint32_t canId = 0;
    uint8_t byteIndex = 0;
    uint8_t beforeValue = 0;
    uint8_t afterValue = 0;
    uint8_t changedBitMask = 0;
    uint16_t changeCount = 0;
    uint8_t confidence = 0;
    uint32_t firstSeenMs = 0;
    uint32_t lastSeenMs = 0;
};

const char* statusText(CapabilityStatus status);
bool isTerminal(CapabilityStatus status);

} // namespace Capabilities
