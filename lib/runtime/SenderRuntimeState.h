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

    static bool hasConsumptionInput() {
        return lastSpeedKph_ >= 1.0f && lastFuelRateLh_ >= 0.1f;
    }

    static float currentConsumptionL100Km() {
        if (!hasConsumptionInput()) return 0.0f;
        return (lastFuelRateLh_ / lastSpeedKph_) * 100.0f;
    }

    static bool addConsumptionSample(float& averageL100Km) {
        const float consumption = currentConsumptionL100Km();
        if (consumption <= 0.0f) return false;

        consumptionSum_ += consumption;
        ++consumptionCount_;

        if (consumptionCount_ < SamplesPerAverage) return false;

        averageL100Km = consumptionSum_ / static_cast<float>(consumptionCount_);
        consumptionSum_ = 0.0f;
        consumptionCount_ = 0;
        return true;
    }

    static float lastSpeedKph() { return lastSpeedKph_; }
    static float lastRpm() { return lastRpm_; }
    static float lastEngineLoadPercent() { return lastEngineLoadPercent_; }
    static float lastThrottlePercent() { return lastThrottlePercent_; }
    static float lastFuelRateLh() { return lastFuelRateLh_; }
    static uint32_t consumptionSampleCount() { return consumptionCount_; }

    static void reset() {
        lastSpeedKph_ = 0.0f;
        lastRpm_ = 0.0f;
        lastEngineLoadPercent_ = 0.0f;
        lastThrottlePercent_ = 0.0f;
        lastFuelRateLh_ = 0.0f;
        consumptionSum_ = 0.0f;
        consumptionCount_ = 0;
    }

private:
    static constexpr uint32_t SamplesPerAverage = 10;
    static inline float lastSpeedKph_ = 0.0f;
    static inline float lastRpm_ = 0.0f;
    static inline float lastEngineLoadPercent_ = 0.0f;
    static inline float lastThrottlePercent_ = 0.0f;
    static inline float lastFuelRateLh_ = 0.0f;
    static inline float consumptionSum_ = 0.0f;
    static inline uint32_t consumptionCount_ = 0;
};

}
