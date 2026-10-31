#include "PidPollPlanner.h"

namespace Obd {

void PidPollPlanner::clear() {
    count_ = 0;
}

bool PidPollPlanner::add(uint8_t pid, uint32_t intervalMs) {
    if (count_ >= Capacity || find(pid) != nullptr || intervalMs == 0) return false;
    entries_[count_++] = Entry{pid, intervalMs, 0, false, true};
    return true;
}

PidPollPlanner::Entry* PidPollPlanner::find(uint8_t pid) {
    for (std::size_t index = 0; index < count_; ++index) {
        if (entries_[index].pid == pid) return &entries_[index];
    }
    return nullptr;
}

void PidPollPlanner::setSupported(uint8_t pid, bool supported) {
    Entry* entry = find(pid);
    if (entry != nullptr) entry->supported = supported;
}

bool PidPollPlanner::next(uint32_t nowMs, uint8_t& pid) const {
    const Entry* best = nullptr;
    // Overdue ratio as fixed point (elapsed * 1024 / interval). Never polled
    // PIDs are treated as maximally overdue, earlier entries win ties.
    uint64_t bestScore = 0;
    for (std::size_t index = 0; index < count_; ++index) {
        const Entry& entry = entries_[index];
        if (!entry.supported) continue;
        const uint64_t score = entry.polledOnce
                                   ? (static_cast<uint64_t>(nowMs - entry.lastPolledMs) << 10U) / entry.intervalMs
                                   : UINT64_MAX;
        if (score >= 1024U && (best == nullptr || score > bestScore)) {
            best = &entry;
            bestScore = score;
        }
    }
    if (best == nullptr) return false;
    pid = best->pid;
    return true;
}

void PidPollPlanner::markPolled(uint8_t pid, uint32_t nowMs) {
    Entry* entry = find(pid);
    if (entry == nullptr) return;
    entry->lastPolledMs = nowMs;
    entry->polledOnce = true;
}

} // namespace Obd
