#pragma once

#include <Arduino.h>

namespace SimulationData {

struct Sample {
    const char* type;
    const char* key;
    const char* name;
    const char* unit;
    float minValue;
    float maxValue;
    uint8_t decimals;
};

// Werte fuer alle Display-Seiten. Die Simulation laeuft deterministisch als
// langsam schwingende Rampe, damit Verbindungs- und Timeoutverhalten testbar ist.
inline constexpr Sample Samples[] = {
    {"OBD", "0D", "Speed", "km/h", 0.0f, 130.0f, 0},
    {"OBD", "0C", "RPM", "rpm", 750.0f, 4200.0f, 0},
    {"OBD", "05", "CoolantTemp", "°C", 75.0f, 104.0f, 0},
    {"BATTERY", "VOLTAGE", "BatteryVoltage", "V", 12.2f, 14.4f, 1},
    {"OBD", "5C", "OilTemp", "°C", 70.0f, 118.0f, 0},
    {"OBD", "04", "EngineLoad", "%", 12.0f, 92.0f, 0},
    {"OBD", "0F", "IntakeTemp", "°C", 18.0f, 55.0f, 0},
    {"FUEL", "AVG", "AverageConsumption", "L/100km", 5.5f, 12.8f, 1},
    {"OBD", "5E", "FuelRate", "L/h", 0.4f, 24.0f, 1},
    {"OBD", "11", "Throttle", "%", 0.0f, 88.0f, 0},
    {"OBD", "10", "MAF", "g/s", 2.0f, 95.0f, 1},
    {"OBD", "2F", "FuelLevel", "%", 18.0f, 84.0f, 0},
    {"OBD", "1F", "RunTime", "s", 0.0f, 3600.0f, 0},
    {"OBD", "46", "AmbientTemp", "°C", -5.0f, 36.0f, 0},
};

inline constexpr size_t SampleCount = sizeof(Samples) / sizeof(Samples[0]);

inline float valueForSample(const Sample& sample, uint32_t millisNow, size_t index) {
    const float phase = ((millisNow / 250U) + static_cast<uint32_t>(index * 17U)) % 200U;
    const float triangle = phase < 100.0f ? phase / 100.0f : (200.0f - phase) / 100.0f;
    return sample.minValue + (sample.maxValue - sample.minValue) * triangle;
}

} // namespace SimulationData
