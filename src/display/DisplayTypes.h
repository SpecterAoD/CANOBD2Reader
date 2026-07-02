#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include "config/ProjectConfig.h"
#include "config/BuildConfig.h"
#include "config/SenderConfig.h"
#include "config/DisplayConfig.h"
#include "config/NetworkConfig.h"
#include "config/SecurityConfig.h"
#include "config/SimulationConfig.h"
#include "config/LoggingConfig.h"
#include "config/ObdConfig.h"
#include "common_config.h"
#include "TelemetryProtocol.h"
#include "DisplaySeverity.h"

#define DISPLAY_FIRMWARE_VERSION CANOBD2_FIRMWARE_VERSION
#define TELEMETRY_PROTOCOL_VERSION CANOBD2_PROTOCOL_VERSION

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
