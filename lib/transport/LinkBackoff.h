#pragma once

// Backoff for the ESP-NOW link when the display does not answer.
//
// After FailuresBeforeBackoff consecutive send failures the link counts as
// offline: only one packet per ProbeIntervalMs is attempted (enough to notice
// the display coming back). The first successful send ends the backoff.

#include <cstdint>

namespace Transport {

class LinkBackoff {
public:
    LinkBackoff(uint8_t failuresBeforeBackoff, uint32_t probeIntervalMs)
        : threshold_(failuresBeforeBackoff), probeIntervalMs_(probeIntervalMs) {}

    // Whether a packet may be sent now. Records the attempt time.
    bool allowSend(uint32_t nowMs) {
        if (!offline()) return true;
        if (probed_ && nowMs - lastProbeMs_ < probeIntervalMs_) {
            ++skipped_;
            return false;
        }
        probed_ = true;
        lastProbeMs_ = nowMs;
        return true;
    }

    void reportResult(bool ok) {
        if (ok) {
            failures_ = 0;
            probed_ = false;
        } else if (failures_ < 255) {
            ++failures_;
        }
    }

    bool offline() const { return failures_ >= threshold_; }
    uint32_t skipped() const { return skipped_; }

private:
    uint8_t threshold_;
    uint32_t probeIntervalMs_;
    uint8_t failures_ = 0;
    uint32_t lastProbeMs_ = 0;
    uint32_t skipped_ = 0;
    bool probed_ = false;
};

} // namespace Transport
