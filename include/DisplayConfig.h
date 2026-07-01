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
constexpr uint8_t PageCount = 9;
constexpr uint8_t MainPageIndex = 0;
constexpr uint32_t LongPressMs = 1200;
constexpr bool EnableGraphPages = true;

constexpr uint32_t RefreshMs = 120;
constexpr uint32_t ForceFullRenderMs = 1200;
constexpr bool UseSegmentValueRenderer = false;
constexpr bool EnableStartupValueOverlay = false;
constexpr uint32_t StartupValueOverlayMs = 10000;
constexpr float SpeedSmoothingAlpha = 0.28f;
constexpr float RpmSmoothingAlpha = 0.20f;
constexpr uint32_t ValueTimeoutMs = 3000;
constexpr uint32_t ConnectionTimeoutMs = 3000;
constexpr uint32_t EspNowTimeoutMs = 3000;
constexpr uint32_t ObdTimeoutMs = 5000;
constexpr uint32_t CanTimeoutMs = 5000;
constexpr uint32_t ButtonDebounceMs = 250;
constexpr std::size_t TelemetryQueueLength = 16;

constexpr float CoolantWarnC = 95.0f;
constexpr float CoolantCriticalC = 105.0f;
constexpr float OilWarnC = 115.0f;
constexpr float OilCriticalC = 125.0f;
constexpr float VoltageWarnLow = 11.8f;
constexpr float VoltageCriticalLow = 11.2f;
constexpr float VoltageWarnHigh = 14.8f;
constexpr float VoltageCriticalHigh = 15.2f;
constexpr uint16_t RpmWarn = 4200;
constexpr uint16_t RpmCritical = 5000;
constexpr uint16_t RpmMin = 0;
constexpr uint16_t RpmMax = 6000;
constexpr float BoostWarnBar = 0.8f;
constexpr float BoostCriticalBar = 1.2f;
}
