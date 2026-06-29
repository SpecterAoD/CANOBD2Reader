#pragma once
#include "Config.h"

// Allgemeines Debug
#define DEBUG_PRINT(...)        do { if (Config::DebugFlag) Serial.print(__VA_ARGS__); } while (0)
#define DEBUG_PRINTLN(...)      do { if (Config::DebugFlag) Serial.println(__VA_ARGS__); } while (0)

// TWAI / CAN Debug
#define TWAI_DEBUG_PRINT(...)   do { if (Config::TwaiDebugFlag) Serial.print(__VA_ARGS__); } while (0)
#define TWAI_DEBUG_PRINTLN(...) do { if (Config::TwaiDebugFlag) Serial.println(__VA_ARGS__); } while (0)

// Power Debug
#define POWER_DEBUG_PRINT(...)  do { if (Config::PowerDebugFlag) Serial.print(__VA_ARGS__); } while (0)
#define POWER_DEBUG_PRINTLN(...) do { if (Config::PowerDebugFlag) Serial.println(__VA_ARGS__); } while (0)

// Serielle Ausgabe allgemein
#define SERIAL_PRINT(...)       do { if (Config::SerialFlag) Serial.print(__VA_ARGS__); } while (0)
#define SERIAL_PRINTLN(...)     do { if (Config::SerialFlag) Serial.println(__VA_ARGS__); } while (0)