#include "SenderLiveData.h"
#include "ConsumptionAccumulator.h"

#include <Arduino.h>

#include "RuntimeSimulation.h"
#include "SenderTelemetry.h"
#include "config/SenderConfig.h"

namespace {
LiveData::LiveValueTable table;
Obd::ConsumptionAccumulator consumption;
uint32_t lastSnapshotAt = 0;
bool simulationWasActive = false;
} // namespace

namespace SenderLiveData {

void reset() {
    table.reset();
    consumption.reset();
    lastSnapshotAt = 0;
    simulationWasActive = false;
}

void update(LiveData::Metric metric, float value) {
    const uint32_t now = millis();
    table.update(metric, value, now);
    if (metric == LiveData::Metric::Speed) consumption.updateSpeed(value, now);
    if (metric == LiveData::Metric::FuelRate) consumption.updateFuelRate(value, now);
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
    if (simulationWasActive != simulationActive) {
        table.reset();
        consumption.reset();
    }
    simulationWasActive = simulationActive;

    if (!simulationActive) {
        consumption.advance(nowMs);
        if (consumption.hasInstant(nowMs)) table.update(LiveData::Metric::InstantConsumption, consumption.instant(), nowMs);
        else table.clear(LiveData::Metric::InstantConsumption);
        if (consumption.fresh(nowMs) && consumption.hasAverage())
            table.update(LiveData::Metric::AverageConsumption, consumption.average(), nowMs);
    }

    if (nowMs - lastSnapshotAt < SenderConfig::LiveSnapshotIntervalMs) return;
    lastSnapshotAt = nowMs;

    uint8_t buffer[LiveData::EncodedSnapshotSize];
    const std::size_t size = LiveData::encodeSnapshot(table.snapshot(nowMs), buffer, sizeof(buffer));
    if (size > 0) SenderTelemetry::sendBinary(Telemetry::PacketType::LiveSnapshot, buffer, size);
}

} // namespace SenderLiveData
