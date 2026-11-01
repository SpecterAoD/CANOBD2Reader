#pragma once

// Rate limiter for repeating events (CAN bus errors, radio send failures).
//
// record() counts every occurrence but returns true at most once per
// interval; summary() then tells how many events happened since the last
// emitted one. This keeps the flash log and the radio link quiet while the
// information ("37x in 5 s") is preserved.

#include <cstdint>

namespace Logging {

class EventThrottle {
public:
    explicit EventThrottle(uint32_t intervalMs) : intervalMs_(intervalMs) {}

    // Returns true if the caller should report now. `count()` is then the
    // number of events including this one since the previous report.
    bool record(uint32_t nowMs) {
        ++pending_;
        if (reported_ && nowMs - lastReportMs_ < intervalMs_) return false;
        reported_ = true;
        lastReportMs_ = nowMs;
        count_ = pending_;
        pending_ = 0;
        return true;
    }

    uint32_t count() const { return count_; }
    uint32_t suppressed() const { return pending_; }

private:
    uint32_t intervalMs_;
    uint32_t lastReportMs_ = 0;
    uint32_t pending_ = 0;
    uint32_t count_ = 0;
    bool reported_ = false;
};

} // namespace Logging
