#include "DisplaySimulation.h"
#include "DisplayData.h"
#include "SimulationData.h"

namespace DisplaySimulation {
  void update() {
    using namespace DisplayData;
    if (!DisplayConfig::EnableInternalSimulation) return;
    if (millis() - lastInternalSimulationUpdate < DisplayConfig::ScreenRefreshMs) return;

    if (internalSimulationIndex == 0) {
      Serial.println("[display-sim] internal simulation active");
    }

    lastInternalSimulationUpdate = millis();

    for (size_t sampleIndex = 0; sampleIndex < SimulationData::SampleCount; ++sampleIndex) {
      const SimulationData::Sample& sample = SimulationData::Samples[sampleIndex];

      char value[16];
      const float simulatedValue =
          SimulationData::valueForSample(sample, millis(), internalSimulationIndex + sampleIndex);
      snprintf(value, sizeof(value), sample.decimals == 0 ? "%.0f" : "%.1f", simulatedValue);
      upsertValue(sample.type,
                  sample.key,
                  sample.name,
                  value,
                  sample.unit,
                  "OK",
                  static_cast<uint32_t>(internalSimulationIndex + sampleIndex + 1));
    }

    internalSimulationIndex += SimulationData::SampleCount;
    lastReceivedAt = millis();
    upsertValue("STATUS", "CAN", "CAN", "SIMULATED", "", "OK", internalSimulationIndex);
    upsertValue("CAN", "RAW", "LastCAN", "0x7E8 DLC8 04 41 0C 1A F8 55 55 55", "", "OK", internalSimulationIndex + 1);
    upsertValue("CAN", "HINT", "CANHint", "Lokale CAN-Simulation", "", "OK", internalSimulationIndex + 2);
    upsertValue("CAN", "COUNT", "CANCount", "128", "frames", "OK", internalSimulationIndex + 3);
    upsertValue("DTC", "ACTIVE", "DTC", "P0133 P0420", "", "WARN", internalSimulationIndex + 4);
  }
}
