#include "SenderLiveData.h"

#include <Arduino.h>

#include "RuntimeSimulation.h"
#include "SenderTelemetry.h"
#include "config/SenderConfig.h"

namespace {
LiveData::LiveValueTable table;
uint32_t lastSnapshotAt = 0;
bool simulationWasActive = false;
} // namespace

namespace SenderLiveData {

void reset() {
    table.reset();
    lastSnapshotAt = 0;
    simulationWasActive = false;
}

void update(LiveData::Metric metric, float value) {
    table.update(metric, value, millis());
}

void markUnsupported(LiveData::Metric metric) {
    table.markUnsupported(metric);
}

void clear(LiveData::Metric metric) {
    table.clear(metric);
}

void setSimulated(bool simulated) {
    table.setSimulated(simulated);
}

bool has(LiveData::Metric metric) {
    return table.has(metric);
}

float value(LiveData::Metric metric) {
    return table.value(metric);
}

void tick(uint32_t nowMs) {
    // Leaving the simulation must not leave simulated values on the display.
    const bool simulationActive = Simulation::RuntimeSimulation::enabled();
    if (simulationWasActive && !simulationActive) table.reset();
    simulationWasActive = simulationActive;

    if (nowMs - lastSnapshotAt < SenderConfig::LiveSnapshotIntervalMs) return;
    lastSnapshotAt = nowMs;

    uint8_t buffer[LiveData::EncodedSnapshotSize];
    const std::size_t size = LiveData::encodeSnapshot(table.snapshot(nowMs), buffer, sizeof(buffer));
    if (size > 0) SenderTelemetry::sendBinary(Telemetry::PacketType::LiveSnapshot, buffer, size);
}

} // namespace SenderLiveData
