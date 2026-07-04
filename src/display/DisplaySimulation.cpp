#include "DisplaySimulation.h"
#include "DisplayData.h"
#include "RuntimeSimulation.h"
#include "ObdSimulation.h"
#include "IsoTpSimulation.h"
#include "SimulationData.h"
#include "DiagnosticLog.h"
#include <cstring>

namespace DisplaySimulation {
  namespace {
    const char* simulatedPowerState(Simulation::Scenario scenario) {
      switch (scenario) {
        case Simulation::Scenario::PowerRunning: return "Running";
        case Simulation::Scenario::PowerStartStop: return "StartStop";
        case Simulation::Scenario::PowerIdle: return "Idle";
        case Simulation::Scenario::PowerParked: return "Parked";
        case Simulation::Scenario::PowerDisplaySleep: return "DisplaySleep";
        case Simulation::Scenario::PowerWakeup: return "Running";
        default: return "Running";
      }
    }

    const char* simulatedPowerCommand(Simulation::Scenario scenario) {
      if (scenario == Simulation::Scenario::PowerDisplaySleep) return "Sleep";
      if (scenario == Simulation::Scenario::PowerWakeup) return "Wakeup";
      return "None";
    }
  }

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
    runtime().lastReceivedAt = millis();
    runtime().lastHeartbeatAt = millis();
    runtime().lastHeartbeatSequence = static_cast<uint32_t>(internalSimulationIndex);
    const auto isoTp = Simulation::buildIsoTpSequence(scenario);
    upsertValue("STATUS", "CAN", "CAN", "SIMULATED", "", "OK", internalSimulationIndex);
    upsertValue("STATUS", "OBD", "OBD", isoTp.timeoutExpected ? "TIMEOUT" : "SIMULATED", "", isoTp.timeoutExpected ? "TIMEOUT" : "OK", internalSimulationIndex + 1);
    runtime().lastCanStatusAt = millis();
    runtime().lastObdStatusAt = millis();
    upsertValue("STATUS", "HEARTBEAT", "Heartbeat", String(static_cast<unsigned long>(internalSimulationIndex)), "", "OK", internalSimulationIndex);
    upsertValue("STATUS", "SIM", "Simulation", Simulation::RuntimeSimulation::enabled() ? "aktiv" : "inaktiv", "", "OK", internalSimulationIndex + 2);
    upsertValue("STATUS", "POWER_STATE", "PowerState", simulatedPowerState(scenario), "", "OK", internalSimulationIndex + 3);
    upsertValue("STATUS", "POWER_COMMAND", "PowerCommand", simulatedPowerCommand(scenario), "", "OK", internalSimulationIndex + 4);
    upsertValue("STATUS", "ACTIVITY_SCORE", "ActivityScore",
                scenario == Simulation::Scenario::PowerParked || scenario == Simulation::Scenario::PowerDisplaySleep ? "0" : "12",
                "", "OK", internalSimulationIndex + 5);
    runtime().vehicleState = simulatedPowerState(scenario);
    runtime().powerCommand = simulatedPowerCommand(scenario);
    runtime().activityScore = scenario == Simulation::Scenario::PowerParked || scenario == Simulation::Scenario::PowerDisplaySleep ? 0 : 12;
    runtime().displaySleepRequested = scenario == Simulation::Scenario::PowerDisplaySleep;
    if (scenario == Simulation::Scenario::PowerWakeup) runtime().displaySleepRequested = false;
    runtime().lastPowerStatusAt = millis();
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
