#include "SenderSimulationScheduler.h"

#include <Arduino.h>
#include <string.h>

#include "IsoTpSimulation.h"
#include "ObdSimulation.h"
#if __has_include("SimulationData.h")
  #include "SimulationData.h"
#else
  #include "../include/SimulationData.h"
#endif
#include "RuntimeSimulation.h"
#include "SenderTelemetry.h"
#include "config/SenderConfig.h"

namespace {
uint32_t lastSimulationSendAt = 0;
size_t simulationSampleIndex = 0;
}

namespace SenderSimulationScheduler {

void reset() {
    lastSimulationSendAt = 0;
    simulationSampleIndex = 0;
}

void tick(uint32_t nowMs) {
    if (nowMs - lastSimulationSendAt < SenderConfig::SimulationIntervalMs) return;
    lastSimulationSendAt = nowMs;

    const auto scenario = Simulation::RuntimeSimulation::scenario();
    const auto simulated = Simulation::simulatedPidValue(simulationSampleIndex, nowMs, scenario);

    char value[16];
    snprintf(value, sizeof(value),
             simulated.decimals == 0 ? "%.0f" : "%.1f",
             static_cast<double>(simulated.value));

    SenderTelemetry::send(simulated.type,
                          simulated.key,
                          simulated.name,
                          strcmp(simulated.status, "OK") == 0 ? value : "N/A",
                          simulated.unit,
                          simulated.status);

    ++simulationSampleIndex;

    // Interleave status, CAN and DTC frames so every display page can be tested
    // without a vehicle or CAN transceiver.
    if (simulationSampleIndex % Simulation::simulatedPidCount() != 0) return;

    const auto isoTp = Simulation::buildIsoTpSequence(scenario);
    SenderTelemetry::sendStatus("CAN", "SIMULATED", "OK");
    SenderTelemetry::sendStatus("OBD",
                                isoTp.timeoutExpected ? "TIMEOUT" : "SIMULATED",
                                isoTp.timeoutExpected ? "TIMEOUT" : "OK");
    SenderTelemetry::sendStatus("SIM", Simulation::RuntimeSimulation::enabled() ? "ACTIVE" : "INACTIVE", "OK");
    SenderTelemetry::send("STATUS", "SIM_SCENARIO", "SimScenario",
                          Simulation::RuntimeSimulation::scenarioName(), "", "OK");
    SenderTelemetry::send("STATUS", "SIM_DETAIL", "SimDetail",
                          Simulation::scenarioDiagnosticText(scenario), "", "OK");
    SenderTelemetry::send("CAN", "RAW", "LastCAN",
                          "0x7E8 DLC8 04 41 0C 1A F8 55 55 55", "", "OK");
    SenderTelemetry::send("CAN", "HINT", "CANHint",
                          Simulation::scenarioDiagnosticText(scenario), "",
                          isoTp.negativeResponse ? "ERROR" : (isoTp.timeoutExpected ? "TIMEOUT" : "OK"));
    SenderTelemetry::send("CAN", "COUNT", "CANCount", "128", "frames", "OK");
    SenderTelemetry::send("DTC", "ACTIVE", "DTC",
                          scenario == Simulation::Scenario::NormalSingleFrame ? "Keine" : "P0133 P0420",
                          "",
                          scenario == Simulation::Scenario::NormalSingleFrame ? "OK" : "WARN");
}

} // namespace SenderSimulationScheduler
