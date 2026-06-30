#pragma once

#include <cstddef>
#include <cstdint>
#include <math.h>
#include <string.h>
#include "BoostCalculator.h"

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
// langsam schwingende Rampe, damit ESP-NOW, CRC, Parser und Anzeige ohne
// Fahrzeug getestet werden koennen.
inline constexpr Sample Samples[] = {
    {"OBD", "0D", "Speed", "km/h", 0.0f, 130.0f, 0},
    {"OBD", "0C", "RPM", "rpm", 750.0f, 4200.0f, 0},
    {"OBD", "05", "CoolantTemp", "°C", 75.0f, 104.0f, 0},
    {"BATTERY", "VOLTAGE", "BatteryVoltage", "V", 12.2f, 14.4f, 1},
    {"OBD", "5C", "OilTemp", "°C", 70.0f, 118.0f, 0},
    {"OBD", "04", "EngineLoad", "%", 12.0f, 92.0f, 0},
    {"OBD", "0B", "ManifoldAbsolutePressure", "kPa", 35.0f, 250.0f, 0},
    {"OBD", "33", "BarometricPressure", "kPa", 90.0f, 105.0f, 0},
    {"OBD", "BOOST", "BoostPressureBar", "bar", -0.6f, 1.6f, 2},
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

inline float clampValue(float value, float minValue, float maxValue) {
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

inline float normalizePhase(uint32_t millisNow, float periodSec) {
    const float seconds = static_cast<float>(millisNow) * 0.001f;
    const float cycles = seconds / periodSec;
    return cycles - floorf(cycles);
}

inline float valueForKey(const char* key, uint32_t millisNow) {
    const float tSec = static_cast<float>(millisNow) * 0.001f;
    const float cycleSec = 130.0f;
    const float cyclePos = fmodf(tSec, cycleSec);

    float speedKmh = 0.0f;
    float throttlePct = 5.0f;

    if (cyclePos < 8.0f) {
        speedKmh = 0.0f;
        throttlePct = 5.0f;
    } else if (cyclePos < 22.0f) {
        const float p = (cyclePos - 8.0f) / 14.0f;
        speedKmh = 70.0f * p;
        throttlePct = 52.0f + 20.0f * sinf(p * 3.14159f);
    } else if (cyclePos < 36.0f) {
        speedKmh = 72.0f + 10.0f * sinf((cyclePos - 22.0f) * 0.62f);
        throttlePct = 42.0f + 15.0f * sinf((cyclePos - 22.0f) * 1.15f);
    } else if (cyclePos < 50.0f) {
        const float p = (cyclePos - 36.0f) / 14.0f;
        speedKmh = 82.0f + 46.0f * p;
        throttlePct = 58.0f + 24.0f * sinf(p * 3.14159f);
    } else if (cyclePos < 70.0f) {
        speedKmh = 124.0f + 8.0f * sinf((cyclePos - 50.0f) * 0.55f);
        throttlePct = 38.0f + 16.0f * sinf((cyclePos - 50.0f) * 0.72f);
    } else if (cyclePos < 84.0f) {
        const float p = (cyclePos - 70.0f) / 14.0f;
        speedKmh = 128.0f - 90.0f * p;
        throttlePct = 24.0f - 13.0f * p;
    } else if (cyclePos < 106.0f) {
        speedKmh = 45.0f + 20.0f * sinf((cyclePos - 84.0f) * 0.88f);
        throttlePct = 33.0f + 18.0f * sinf((cyclePos - 84.0f) * 1.35f);
    } else if (cyclePos < 122.0f) {
        const float p = (cyclePos - 106.0f) / 16.0f;
        speedKmh = 42.0f * (1.0f - p);
        throttlePct = 11.0f * (1.0f - p);
    } else {
        speedKmh = 0.0f;
        throttlePct = 5.0f;
    }

    speedKmh = clampValue(speedKmh, 0.0f, 130.0f);
    throttlePct = clampValue(throttlePct, 0.0f, 92.0f);

    const float runtimeSec = floorf(tSec);
    const float ambientTemp = 14.0f + 7.5f * sinf(tSec * 0.003f);
    const float loadPct = clampValue(9.0f + throttlePct * 1.05f + speedKmh * 0.24f, 8.0f, 99.0f);
    const float idleWave = sinf(tSec * 2.3f) * 55.0f;

    float rpm = 820.0f;
    if (speedKmh < 1.0f) {
        rpm = 780.0f + idleWave;
    } else {
        float gearFactor = 42.0f;
        if (speedKmh > 108.0f) {
            gearFactor = 31.0f;
        } else if (speedKmh > 72.0f) {
            gearFactor = 35.0f;
        }
        rpm = 920.0f + speedKmh * gearFactor + throttlePct * 26.0f + sinf(tSec * 1.55f) * 210.0f;
    }
    rpm = clampValue(rpm, 720.0f, 4500.0f);

    const float warmup = 1.0f - expf(-tSec / 220.0f);
    const float coolantTemp = clampValue(24.0f + warmup * 68.0f + loadPct * 0.09f, 72.0f, 108.0f);
    const float oilTemp = clampValue(coolantTemp - 3.0f + loadPct * 0.18f + sinf(tSec * 0.08f) * 2.8f, 70.0f, 122.0f);
    const float intakeTemp = clampValue(ambientTemp + 5.0f + loadPct * 0.11f - speedKmh * 0.04f, -5.0f, 58.0f);
    const float maf = clampValue((rpm / 60.0f) * (loadPct / 100.0f) * 1.6f + 1.5f, 2.0f, 99.0f);
    const float barometricKpa = 101.0f + 1.5f * sinf(tSec * 0.002f);
    const float manifoldKpa = clampValue(34.0f + loadPct * 1.72f + throttlePct * 0.42f, 35.0f, 250.0f);
    const float boostBar = Obd::calculateBoostPressureBar(manifoldKpa, barometricKpa);

    float fuelRate = 0.75f + throttlePct * 0.05f;
    if (speedKmh > 2.0f) {
        fuelRate = 1.25f + loadPct * 0.11f + speedKmh * 0.024f;
    }
    fuelRate = clampValue(fuelRate, 0.4f, 24.0f);

    float avgConsumption = 6.3f + loadPct * 0.056f;
    if (speedKmh < 25.0f) avgConsumption += 2.2f;
    if (speedKmh > 95.0f) avgConsumption += 1.4f;
    avgConsumption = clampValue(avgConsumption, 5.2f, 12.9f);

    const float batteryV = clampValue(13.95f - throttlePct * 0.004f - (rpm < 900.0f ? 0.55f : 0.0f)
                                      + sinf(tSec * 0.9f) * 0.06f,
                                      12.2f, 14.4f);
    const float fuelLevel = clampValue(82.0f - runtimeSec / 90.0f, 18.0f, 84.0f);

    if (strcmp(key, "0D") == 0) return speedKmh;
    if (strcmp(key, "0C") == 0) return rpm;
    if (strcmp(key, "05") == 0) return coolantTemp;
    if (strcmp(key, "VOLTAGE") == 0) return batteryV;
    if (strcmp(key, "5C") == 0) return oilTemp;
    if (strcmp(key, "04") == 0) return loadPct;
    if (strcmp(key, "0B") == 0) return manifoldKpa;
    if (strcmp(key, "33") == 0) return barometricKpa;
    if (strcmp(key, "BOOST") == 0) return boostBar;
    if (strcmp(key, "0F") == 0) return intakeTemp;
    if (strcmp(key, "AVG") == 0) return avgConsumption;
    if (strcmp(key, "5E") == 0) return fuelRate;
    if (strcmp(key, "11") == 0) return throttlePct;
    if (strcmp(key, "10") == 0) return maf;
    if (strcmp(key, "2F") == 0) return fuelLevel;
    if (strcmp(key, "1F") == 0) return runtimeSec;
    if (strcmp(key, "46") == 0) return ambientTemp;

    return 0.0f;
}

inline float valueForSample(const Sample& sample, uint32_t millisNow, size_t index) {
    const float keyedValue = valueForKey(sample.key, millisNow);
    if (keyedValue != 0.0f || strcmp(sample.key, "0D") == 0 || strcmp(sample.key, "1F") == 0) {
        return clampValue(keyedValue, sample.minValue, sample.maxValue);
    }

    const float phase = normalizePhase(millisNow + static_cast<uint32_t>(index * 137U), 24.0f);
    const float waveform = 0.5f + 0.5f * sinf(phase * 6.28318f);
    return sample.minValue + (sample.maxValue - sample.minValue) * waveform;
}

} // namespace SimulationData
