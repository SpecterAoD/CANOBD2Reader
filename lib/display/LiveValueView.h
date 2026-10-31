#pragma once

// Display-side view of the live snapshot.
//
// Each value is classified for rendering:
//   Fresh       - recent enough to be trusted (normal colour)
//   Stale       - older than its fresh limit but < MaxStaleMs: last value is
//                 still shown, greyed out. A single missed poll therefore
//                 never blanks a value ("flicker" of the old firmware).
//   Unsupported - the ECU does not provide this PID
//   Missing     - never received or older than MaxStaleMs
//
// Value age = time since the snapshot arrived + age reported by the sender, so
// a lost radio link also turns values stale automatically.

#include <cstdint>

#include "LiveData.h"

namespace DisplayLogic {

enum class ValueState : uint8_t { Missing, Unsupported, Stale, Fresh };

struct ValueView {
    ValueState state = ValueState::Missing;
    float value = 0.0f;
    uint32_t ageMs = 0;
    bool hasValue() const { return state == ValueState::Fresh || state == ValueState::Stale; }
};

class LiveValueView {
public:
    static constexpr uint32_t MaxStaleMs = 60000;

    void reset();
    void apply(const LiveData::Snapshot& snapshot, uint32_t receivedAtMs);
    ValueView get(LiveData::Metric metric, uint32_t nowMs) const;
    bool simulated() const { return simulated_; }
    bool everReceived() const { return received_; }
    uint32_t lastSnapshotAt() const { return lastSnapshotAt_; }

    // How old a value may be before it is shown as stale.
    static uint32_t freshLimitMs(LiveData::Metric metric);

private:
    float values_[LiveData::MetricCount]{};
    uint32_t measuredAt_[LiveData::MetricCount]{};
    uint32_t validMask_ = 0;
    uint32_t unsupportedMask_ = 0;
    uint32_t lastSnapshotAt_ = 0;
    bool simulated_ = false;
    bool received_ = false;
};

// Exponential smoothing for fast moving numbers (needle/number animation).
// Time based, so the result does not depend on the render frame rate.
class Smoother {
public:
    explicit Smoother(float timeConstantMs = 150.0f) : tau_(timeConstantMs) {}
    float update(float target, uint32_t nowMs);
    void reset() { initialized_ = false; }

private:
    float tau_;
    float value_ = 0.0f;
    uint32_t lastMs_ = 0;
    bool initialized_ = false;
};

} // namespace DisplayLogic
