#pragma once

#include <cstddef>
#include <cstdint>

namespace DisplayConfigValues {
constexpr uint8_t Rotation = 1;
constexpr int8_t PowerPin = 15;
constexpr uint8_t PowerOnLevel = 1;
constexpr uint16_t PowerStabilizeMs = 60;
constexpr uint8_t BacklightPin = 38;
constexpr uint8_t BacklightOn = 255;
constexpr uint8_t NextPageButtonPin = 0;
constexpr uint8_t PageCount = 7;

constexpr uint32_t RefreshMs = 120;
constexpr uint32_t ForceFullRenderMs = 1200;
constexpr bool UseSegmentValueRenderer = false;
constexpr bool EnableStartupValueOverlay = false;
constexpr uint32_t StartupValueOverlayMs = 10000;
constexpr float SpeedSmoothingAlpha = 0.28f;
constexpr float RpmSmoothingAlpha = 0.20f;
constexpr uint32_t ValueTimeoutMs = 5000;
constexpr uint32_t ConnectionTimeoutMs = 3000;
constexpr uint32_t ButtonDebounceMs = 250;
constexpr std::size_t TelemetryQueueLength = 16;
}
