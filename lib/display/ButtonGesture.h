#pragma once

// Debounced push button with short / long / very long press detection.
//
// The raw input must be stable for DebounceMs before a change counts, so a
// normal quick tap (50-200 ms) is a short press. Long press is reported on
// release; very long press already while holding (immediate feedback).

#include <cstdint>

namespace DisplayLogic {

enum class ButtonEvent : uint8_t { None, Short, Long, VeryLong };

class ButtonGesture {
public:
    ButtonGesture(uint32_t debounceMs, uint32_t longMs, uint32_t veryLongMs)
        : debounceMs_(debounceMs), longMs_(longMs), veryLongMs_(veryLongMs) {}

    // Call every loop with the raw "pressed" level.
    ButtonEvent update(bool rawPressed, uint32_t nowMs);
    bool pressed() const { return stable_; }

private:
    uint32_t debounceMs_;
    uint32_t longMs_;
    uint32_t veryLongMs_;
    bool raw_ = false;
    bool stable_ = false;
    bool veryLongReported_ = false;
    uint32_t rawChangedAt_ = 0;
    uint32_t pressedAt_ = 0;
};

} // namespace DisplayLogic
