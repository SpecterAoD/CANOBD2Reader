#include "DisplaySimulation.h"
#include "DisplayData.h"
#include "RuntimeSimulation.h"
#include "ObdSimulation.h"
#include "IsoTpSimulation.h"
#include "SimulationData.h"
#include "DiagnosticLog.h"
#include <cstring>

namespace DisplaySimulation {
  void update() {
    using namespace DisplayData;
    if (!Simulation::RuntimeSimulation::enabled()) return;
    if (millis() - lastInternalSimulationUpdate < DisplayConfig::ScreenRefreshMs) return;

    if (internalSimulationIndex == 0) {
      DiagnosticLog::appendf("[display-sim] internal simulation active");
    }

    lastInternalSimulationUpdate = millis();

    const auto scenario = Simulation::RuntimeSimulation::scenario();
    for (size_t sampleIndex = 0; sampleIndex < Simulation::simulatedPidCount(); ++sampleIndex) {
      const auto sample = Simulation::simulatedPidValue(sampleIndex, millis(), scenario);

      char value[16];
      snprintf(value, sizeof(value), sample.decimals == 0 ? "%.0f" : "%.1f", static_cast<double>(sample.value));
      upsertValue(sample.type,
                  sample.key,
                  sample.name,
                  strcmp(sample.status, "OK") == 0 ? value : "N/A",
                  sample.unit,
                  sample.status,
                  static_cast<uint32_t>(internalSimulationIndex + sampleIndex + 1));
    }

    internalSimulationIndex += Simulation::simulatedPidCount();
    lastReceivedAt = millis();
    lastHeartbeatAt = millis();
    lastHeartbeatSequence = static_cast<uint32_t>(internalSimulationIndex);
    const auto isoTp = Simulation::buildIsoTpSequence(scenario);
    upsertValue("STATUS", "CAN", "CAN", "SIMULATED", "", "OK", internalSimulationIndex);
    upsertValue("STATUS", "OBD", "OBD", isoTp.timeoutExpected ? "TIMEOUT" : "SIMULATED", "", isoTp.timeoutExpected ? "TIMEOUT" : "OK", internalSimulationIndex + 1);
    lastCanStatusAt = millis();
    lastObdStatusAt = millis();
    upsertValue("STATUS", "HEARTBEAT", "Heartbeat", String(static_cast<unsigned long>(internalSimulationIndex)), "", "OK", internalSimulationIndex);
    upsertValue("STATUS", "SIM", "Simulation", Simulation::RuntimeSimulation::enabled() ? "aktiv" : "inaktiv", "", "OK", internalSimulationIndex + 2);
    upsertValue("STATUS", "SIM_SCENARIO", "SimScenario", Simulation::RuntimeSimulation::scenarioName(), "", "OK", internalSimulationIndex + 3);
    upsertValue("CAN", "RAW", "LastCAN", "0x7E8 DLC8 04 41 0C 1A F8 55 55 55", "", "OK", internalSimulationIndex + 4);
    upsertValue("CAN", "HINT", "CANHint", Simulation::scenarioDiagnosticText(scenario), "", isoTp.negativeResponse ? "ERROR" : (isoTp.timeoutExpected ? "TIMEOUT" : "OK"), internalSimulationIndex + 5);
    upsertValue("CAN", "COUNT", "CANCount", "128", "frames", "OK", internalSimulationIndex + 3);
    upsertValue("DTC", "ACTIVE", "DTC",
                scenario == Simulation::Scenario::NormalSingleFrame ? "Keine" : "P0133 P0420",
                "",
                scenario == Simulation::Scenario::NormalSingleFrame ? "OK" : "WARN",
                internalSimulationIndex + 6);
  }
}
