#pragma once

#include <cmath>
#include <cstdint>
#include "config/SenderConfig.h"

namespace Obd {
// Integrates last-known speed and fuel rate only while BOTH inputs are fresh.
// Input events integrate the preceding interval before replacing a value.
class ConsumptionAccumulator {
public:
    static constexpr uint32_t MaxInputAgeMs = SenderConfig::FastPidIntervalMs * 3 + 900;
    void reset() { *this = ConsumptionAccumulator{}; }
    void updateSpeed(float speedKph, uint32_t nowMs) {
        advance(nowMs);
        speed_ = speedKph;
        speedAt_ = nowMs;
        speedValid_ = std::isfinite(speedKph) && speedKph >= 0;
    }
    void updateFuelRate(float litresPerHour, uint32_t nowMs) {
        advance(nowMs);
        fuel_ = litresPerHour;
        fuelAt_ = nowMs;
        fuelValid_ = std::isfinite(litresPerHour) && litresPerHour >= 0;
    }
    void advance(uint32_t nowMs) {
        if (started_ && speedValid_ && fuelValid_) {
            const uint32_t speedAge = lastAt_ - speedAt_;
            const uint32_t fuelAge = lastAt_ - fuelAt_;
            if (speedAge < MaxInputAgeMs && fuelAge < MaxInputAgeMs) {
                uint32_t dt = nowMs - lastAt_;
                if (dt > MaxInputAgeMs - speedAge) dt = MaxInputAgeMs - speedAge;
                if (dt > MaxInputAgeMs - fuelAge) dt = MaxInputAgeMs - fuelAge;
                const double hours = static_cast<double>(dt) / 3600000.0;
                litres_ += static_cast<double>(fuel_) * hours;
                kilometres_ += static_cast<double>(speed_) * hours;
            }
        }
        started_ = true;
        lastAt_ = nowMs;
    }
    bool fresh(uint32_t nowMs) const {
        return speedValid_ && fuelValid_ && nowMs - speedAt_ <= MaxInputAgeMs &&
               nowMs - fuelAt_ <= MaxInputAgeMs;
    }
    bool hasAverage() const { return kilometres_ >= 0.001; }
    float average() const { return hasAverage() ? static_cast<float>(100.0 * litres_ / kilometres_) : 0.0f; }
    bool hasInstant(uint32_t nowMs) const { return fresh(nowMs) && speed_ >= 5.0f; }
    float instant() const { return speed_ >= 5.0f ? fuel_ * 100.0f / speed_ : 0.0f; }
    double litres() const { return litres_; }
    double kilometres() const { return kilometres_; }

private:
    double litres_ = 0;
    double kilometres_ = 0;
    float speed_ = 0;
    float fuel_ = 0;
    uint32_t speedAt_ = 0;
    uint32_t fuelAt_ = 0;
    uint32_t lastAt_ = 0;
    bool started_ = false;
    bool speedValid_ = false;
    bool fuelValid_ = false;
};
} // namespace Obd
