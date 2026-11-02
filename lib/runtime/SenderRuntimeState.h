#pragma once

#include <cstdint>

namespace Runtime {

class SenderRuntimeState {
public:
    static void updateSpeed(float speedKph) {
        lastSpeedKph_ = speedKph;
    }

    static void updateRpm(float rpm) {
        lastRpm_ = rpm;
    }

    static void updateEngineLoad(float percent) {
        lastEngineLoadPercent_ = percent;
    }

    static void updateThrottle(float percent) {
        lastThrottlePercent_ = percent;
    }

    static void updateFuelRate(float litersPerHour) {
        lastFuelRateLh_ = litersPerHour;
    }

    static float lastSpeedKph() { return lastSpeedKph_; }
    static float lastRpm() { return lastRpm_; }
    static float lastEngineLoadPercent() { return lastEngineLoadPercent_; }
    static float lastThrottlePercent() { return lastThrottlePercent_; }
    static float lastFuelRateLh() { return lastFuelRateLh_; }

    static void reset() {
        lastSpeedKph_ = 0.0f;
        lastRpm_ = 0.0f;
        lastEngineLoadPercent_ = 0.0f;
        lastThrottlePercent_ = 0.0f;
        lastFuelRateLh_ = 0.0f;
    }

private:
    static inline float lastSpeedKph_ = 0.0f;
    static inline float lastRpm_ = 0.0f;
    static inline float lastEngineLoadPercent_ = 0.0f;
    static inline float lastThrottlePercent_ = 0.0f;
    static inline float lastFuelRateLh_ = 0.0f;
};

}
