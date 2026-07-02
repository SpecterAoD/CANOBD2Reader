#pragma once

#include <cstdint>
#include "config/SenderConfig.h"

namespace Obd {

constexpr float DefaultBarometricPressureKpa = SenderConfig::DefaultBarometricPressureKpa;

/// Returns relative boost pressure in kPa. MAP is absolute pressure, therefore
/// boost must always be calculated against ambient/barometric pressure.
inline float calculateBoostPressureKpa(float manifoldAbsolutePressureKpa,
                                       float barometricPressureKpa) {
    return manifoldAbsolutePressureKpa - barometricPressureKpa;
}

/// Returns relative boost pressure in bar. Negative values are kept so vacuum
/// conditions remain visible to diagnostics instead of being silently clipped.
inline float calculateBoostPressureBar(float manifoldAbsolutePressureKpa,
                                       float barometricPressureKpa) {
    return calculateBoostPressureKpa(manifoldAbsolutePressureKpa, barometricPressureKpa) / 100.0f;
}

/// Uses the configured 101.3 kPa fallback when the vehicle does not support
/// PID 0x33 (barometric pressure).
inline float calculateBoostPressureBarWithFallback(float manifoldAbsolutePressureKpa,
                                                   bool hasBarometricPressure,
                                                   float barometricPressureKpa) {
    const float ambientKpa = hasBarometricPressure ? barometricPressureKpa : DefaultBarometricPressureKpa;
    return calculateBoostPressureBar(manifoldAbsolutePressureKpa, ambientKpa);
}

}
