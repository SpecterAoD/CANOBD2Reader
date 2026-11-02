#include "LiveValueView.h"
#include "config/SenderConfig.h"

namespace DisplayLogic {

using LiveData::Metric;

void LiveValueView::reset() {
    *this = LiveValueView{};
}

void LiveValueView::apply(const LiveData::Snapshot& snapshot, uint32_t receivedAtMs) {
    validMask_ = snapshot.validMask;
    unsupportedMask_ = snapshot.unsupportedMask;
    simulated_ = (snapshot.flags & LiveData::FlagSimulated) != 0;
    lastSnapshotAt_ = receivedAtMs;
    received_ = true;
    for (std::size_t index = 0; index < LiveData::MetricCount; ++index) {
        if (!((validMask_ >> index) & 1U)) continue;
        values_[index] = snapshot.values[index];
        const uint32_t ageMs = static_cast<uint32_t>(snapshot.ageTenths[index]) * 100U;
        measuredAt_[index] = receivedAtMs - ageMs;
    }
}

ValueView LiveValueView::get(Metric metric, uint32_t nowMs) const {
    ValueView view;
    const uint8_t index = static_cast<uint8_t>(metric);
    if (index >= LiveData::MetricCount) return view;
    if ((unsupportedMask_ >> index) & 1U) {
        view.state = ValueState::Unsupported;
        return view;
    }
    if (!((validMask_ >> index) & 1U)) return view;

    view.value = values_[index];
    view.ageMs = nowMs - measuredAt_[index];
    if (view.ageMs <= freshLimitMs(metric)) {
        view.state = ValueState::Fresh;
    } else if (view.ageMs <= MaxStaleMs) {
        view.state = ValueState::Stale;
    }
    return view;
}

uint32_t LiveValueView::freshLimitMs(Metric metric) {
    switch (metric) {
        case Metric::Rpm:
        case Metric::Speed:
        case Metric::EngineLoad:
        case Metric::Maf:
        case Metric::FuelRate:
        case Metric::InstantConsumption:
            return SenderConfig::FastPidIntervalMs * 3 + 900;
        case Metric::BatteryVoltage:
            return SenderConfig::BatterySendIntervalMs * 2 + 2000;
        default:
            return SenderConfig::SlowPidIntervalMs * 3 + 1000;
    }
}

float Smoother::update(float target, uint32_t nowMs) {
    if (!initialized_) {
        initialized_ = true;
        value_ = target;
        lastMs_ = nowMs;
        return value_;
    }
    const float dt = static_cast<float>(nowMs - lastMs_);
    lastMs_ = nowMs;
    const float alpha = dt >= tau_ ? 1.0f : dt / tau_;
    value_ += (target - value_) * alpha;
    return value_;
}

} // namespace DisplayLogic
