#pragma once

#include <cstddef>
#include <cstdint>

namespace SenderConfig {
constexpr int CanRxPin = 4;
constexpr int CanTxPin = 5;
constexpr uint32_t CanBitrate = 500000;
constexpr uint32_t ObdPollIntervalMs = 200;
constexpr uint32_t ObdResponseTimeoutMs = 300;
constexpr uint32_t ObdTxTimeoutMs = 50;
constexpr uint32_t SupportedPidRefreshMs = 60000;
constexpr uint32_t DtcQueryIntervalMs = 30000;
constexpr std::size_t CanRxQueueLength = 32;
constexpr std::size_t CanTxQueueLength = 4;
constexpr uint8_t IsoTpBlockSize = 0;
constexpr uint8_t IsoTpStMinMs = 0;
constexpr uint32_t IsoTpConsecutiveFrameTimeoutMs = 300;
}
