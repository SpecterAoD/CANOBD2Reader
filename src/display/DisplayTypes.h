#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "Config.h"
#include "common_config.h"
#include "TelemetryProtocol.h"
#include "DisplaySeverity.h"

#define DISPLAY_FIRMWARE_VERSION CANOBD2_FIRMWARE_VERSION
#define TELEMETRY_PROTOCOL_VERSION CANOBD2_PROTOCOL_VERSION

namespace DisplayConfig {
  constexpr uint8_t Rotation = Config::Display::Rotation;
  constexpr int8_t PowerPin = Config::Display::PowerPin;
  constexpr uint8_t PowerOnLevel = Config::Display::PowerOnLevel;
  constexpr uint16_t PowerStabilizeMs = Config::Display::PowerStabilizeMs;
  constexpr uint8_t BacklightPin = Config::Display::BacklightPin;
  constexpr uint8_t NextPageButtonPin = Config::Display::NextPageButtonPin;
  constexpr bool EnableInternalSimulation = Config::Feature::EnableDisplayInternalSimulation;
  constexpr uint32_t ScreenRefreshMs = Config::Display::ScreenRefreshMs;
  constexpr uint32_t ForceFullRenderMs = Config::Display::ForceFullRenderMs;
  constexpr bool UseSegmentValueRenderer = Config::Display::UseSegmentValueRenderer;
  constexpr bool EnableStartupValueOverlay = Config::Display::EnableStartupValueOverlay;
  constexpr uint32_t StartupValueOverlayMs = Config::Display::StartupValueOverlayMs;
  constexpr float SpeedSmoothingAlpha = Config::Display::SpeedSmoothingAlpha;
  constexpr float RpmSmoothingAlpha = Config::Display::RpmSmoothingAlpha;
  constexpr uint32_t ConnectionTimeoutMs = Config::Display::ConnectionTimeoutMs;
  constexpr uint32_t ValueTimeoutMs = Config::Display::ValueTimeoutMs;
  constexpr uint32_t ButtonDebounceMs = Config::Display::ButtonDebounceMs;
  constexpr uint8_t BacklightOn = Config::Display::BacklightOn;
  constexpr uint8_t PageCount = Config::Display::PageCount;
  constexpr uint8_t MainPageIndex = Config::Display::MainPageIndex;
  constexpr uint32_t LongPressMs = Config::Display::LongPressMs;
  constexpr bool EnableGraphPages = Config::Display::EnableGraphPages;
  constexpr float CoolantWarnC = Config::Display::CoolantWarnC;
  constexpr float CoolantCriticalC = Config::Display::CoolantCriticalC;
  constexpr float OilWarnC = Config::Display::OilWarnC;
  constexpr float OilCriticalC = Config::Display::OilCriticalC;
  constexpr float VoltageWarnLow = Config::Display::VoltageWarnLow;
  constexpr float VoltageCriticalLow = Config::Display::VoltageCriticalLow;
  constexpr float VoltageWarnHigh = Config::Display::VoltageWarnHigh;
  constexpr float VoltageCriticalHigh = Config::Display::VoltageCriticalHigh;
  constexpr uint16_t RpmWarn = Config::Display::RpmWarn;
  constexpr uint16_t RpmCritical = Config::Display::RpmCritical;

  constexpr uint16_t Background = TFT_BLACK;
  constexpr uint16_t Panel = 0x18E3;
  constexpr uint16_t Text = TFT_WHITE;
  constexpr uint16_t Muted = TFT_DARKGREY;
  constexpr uint16_t Accent = TFT_CYAN;
  constexpr uint16_t Ok = TFT_GREEN;
  constexpr uint16_t Warn = TFT_ORANGE;
  constexpr uint16_t Error = TFT_RED;
}

struct DisplayTelemetryValue {
  String type;
  String key;
  String name;
  String value;
  String unit;
  String status;
  uint32_t sequence = 0;
  uint32_t updatedAt = 0;
};
