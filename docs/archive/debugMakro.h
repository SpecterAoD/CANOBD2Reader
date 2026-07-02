#pragma once
// Legacy archive: Config.h was removed during config split.

// Allgemeines Debug
#define DEBUG_PRINT(...)        do { if (LoggingLegacyConfig::GeneralDebugEnabled) Serial.print(__VA_ARGS__); } while (0)
#define DEBUG_PRINTLN(...)      do { if (LoggingLegacyConfig::GeneralDebugEnabled) Serial.println(__VA_ARGS__); } while (0)

// TWAI / CAN Debug
#define TWAI_DEBUG_PRINT(...)   do { if (LoggingLegacyConfig::TwaiDebugEnabled) Serial.print(__VA_ARGS__); } while (0)
#define TWAI_DEBUG_PRINTLN(...) do { if (LoggingLegacyConfig::TwaiDebugEnabled) Serial.println(__VA_ARGS__); } while (0)

// Power Debug
#define POWER_DEBUG_PRINT(...)  do { if (LoggingLegacyConfig::PowerDebugEnabled) Serial.print(__VA_ARGS__); } while (0)
#define POWER_DEBUG_PRINTLN(...) do { if (LoggingLegacyConfig::PowerDebugEnabled) Serial.println(__VA_ARGS__); } while (0)

// Serielle Ausgabe allgemein
#define SERIAL_PRINT(...)       do { if (LoggingLegacyConfig::SerialEnabled) Serial.print(__VA_ARGS__); } while (0)
#define SERIAL_PRINTLN(...)     do { if (LoggingLegacyConfig::SerialEnabled) Serial.println(__VA_ARGS__); } while (0)