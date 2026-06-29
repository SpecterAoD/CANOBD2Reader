#pragma once

#include <cstddef>
#include <cstdint>

namespace DisplayConfigValues {
constexpr uint32_t RefreshMs = 120;
constexpr uint32_t ValueTimeoutMs = 5000;
constexpr uint32_t ConnectionTimeoutMs = 3000;
constexpr std::size_t TelemetryQueueLength = 16;
constexpr int PowerPin = 15;
constexpr int BacklightPin = 38;
constexpr int ButtonPin = 0;
}
