#pragma once

// Fair OBD Mode 01 polling.
//
// Every PID has a target interval (fast values ~200 ms, slow ones ~2 s). next()
// returns the PID that is most overdue relative to its own interval, so fast
// values stay fast while slow values are still guaranteed a slot. A PID that
// the ECU reports as unsupported is skipped.

#include <cstddef>
#include <cstdint>

namespace Obd {

class PidPollPlanner {
public:
    static constexpr std::size_t Capacity = 24;

    void clear();
    bool add(uint8_t pid, uint32_t intervalMs);
    void setSupported(uint8_t pid, bool supported);
    // Picks the next PID to query; false if nothing is due yet.
    bool next(uint32_t nowMs, uint8_t& pid) const;
    void markPolled(uint8_t pid, uint32_t nowMs);
    std::size_t size() const { return count_; }

private:
    struct Entry {
        uint8_t pid;
        uint32_t intervalMs;
        uint32_t lastPolledMs;
        bool polledOnce;
        bool supported;
    };
    Entry* find(uint8_t pid);

    Entry entries_[Capacity]{};
    std::size_t count_ = 0;
};

} // namespace Obd
