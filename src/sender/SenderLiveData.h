#pragma once

// Sender side of the live snapshot (telemetry protocol v3): collects the
// latest value of every metric and broadcasts the whole table periodically.
#include <cstdint>

#include "LiveData.h"

namespace SenderLiveData {

void reset();
void update(LiveData::Metric metric, float value);
void markUnsupported(LiveData::Metric metric);
void clear(LiveData::Metric metric);
void setSimulated(bool simulated);
bool has(LiveData::Metric metric);
float value(LiveData::Metric metric);
// Sends a snapshot when SenderConfig::LiveSnapshotIntervalMs elapsed.
void tick(uint32_t nowMs);

} // namespace SenderLiveData
