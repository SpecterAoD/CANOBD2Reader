#pragma once

#include <Arduino.h>

#include "common_config.h"
#include "config/DisplayConfig.h"
#include "config/LoggingConfig.h"
#include "config/ProjectConfig.h"

// One status/diagnostic text value (CAN state, DTC list, VIN, power state ...).
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
