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
size_t simulationStatusIndex = 0;

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

const char* simulatedActivityScore(Simulation::Scenario scenario) {
    switch (scenario) {
        case Simulation::Scenario::PowerRunning: return "17";
        case Simulation::Scenario::PowerStartStop: return "12";
        case Simulation::Scenario::PowerIdle: return "6";
        case Simulation::Scenario::PowerParked: return "0";
        case Simulation::Scenario::PowerDisplaySleep: return "0";
        case Simulation::Scenario::PowerWakeup: return "17";
        default: return "12";
    }
}

constexpr size_t kSimulationStatusFrameCount = 19;

void sendSimulationStatusFrame(size_t index, Simulation::Scenario scenario) {
    const auto isoTp = Simulation::buildIsoTpSequence(scenario);
    switch (index % kSimulationStatusFrameCount) {
        case 0:
            SenderTelemetry::sendStatus("CAN", "SIMULATED", "OK");
            break;
        case 1:
            SenderTelemetry::sendStatus("OBD",
                                        isoTp.timeoutExpected ? "TIMEOUT" : "SIMULATED",
                                        isoTp.timeoutExpected ? "TIMEOUT" : "OK");
            break;
        case 2:
            SenderTelemetry::sendStatus("SIM",
                                        Simulation::RuntimeSimulation::enabled() ? "ACTIVE" : "INACTIVE",
                                        "OK");
            break;
        case 3:
            SenderTelemetry::sendStatus("POWER_STATE", simulatedPowerState(scenario), "OK");
            break;
        case 4:
            SenderTelemetry::sendStatus("ACTIVITY_SCORE", simulatedActivityScore(scenario), "OK");
            break;
        case 5:
            SenderTelemetry::sendStatus("POWER_COMMAND", simulatedPowerCommand(scenario), "OK");
            break;
        case 6:
            SenderTelemetry::sendStatus("UDS", "SIMULATED", "OK");
            break;
        case 7:
            SenderTelemetry::send("STATUS", "ECU_COUNT", "ReachableEcus", "2", "", "OK");
            break;
        case 8:
            SenderTelemetry::send("STATUS", "UDS_PENDING", "UdsPending",
                                  scenario == Simulation::Scenario::NormalMultiFrameDtc ? "1" : "0",
                                  "",
                                  scenario == Simulation::Scenario::NormalMultiFrameDtc ? "WARN" : "OK");
            break;
        case 9:
            SenderTelemetry::send("STATUS", "UDS_BACKOFF", "UdsBackoff", "0s", "", "OK");
            break;
        case 10:
            SenderTelemetry::send("STATUS", "UDS_DID_F190", "UDS_VIN", "WVGZZZ5N6RM079696", "", "OK");
            break;
        case 11:
            SenderTelemetry::send("DTC", "UDS", "UDS_DTC",
                                  scenario == Simulation::Scenario::NormalMultiFrameDtc ? "P0133 P0420" : "Keine",
                                  "",
                                  scenario == Simulation::Scenario::NormalMultiFrameDtc ? "WARN" : "OK");
            break;
        case 12:
            SenderTelemetry::send("STATUS", "UDS_NEGATIVE", "UDS_NRC",
                                  scenario == Simulation::Scenario::NormalMultiFrameDtc ? "0x78 ResponsePending" : "--",
                                  "",
                                  scenario == Simulation::Scenario::NormalMultiFrameDtc ? "WARN" : "OK");
            break;
        case 13:
            SenderTelemetry::send("STATUS", "SIM_SCENARIO", "SimScenario",
                                  Simulation::RuntimeSimulation::scenarioName(), "", "OK");
            break;
        case 14:
            SenderTelemetry::send("STATUS", "SIM_DETAIL", "SimDetail",
                                  Simulation::scenarioDiagnosticText(scenario), "", "OK");
            break;
        case 15:
            SenderTelemetry::send("CAN", "RAW", "LastCAN",
                                  "0x7E8 DLC8 04 41 0C 1A F8 55 55 55", "", "OK");
            break;
        case 16:
            SenderTelemetry::send("CAN", "HINT", "CANHint",
                                  Simulation::scenarioDiagnosticText(scenario), "",
                                  isoTp.negativeResponse ? "ERROR" : (isoTp.timeoutExpected ? "TIMEOUT" : "OK"));
            break;
        case 17:
            SenderTelemetry::send("CAN", "COUNT", "CANCount", "128", "frames", "OK");
            break;
        case 18:
            SenderTelemetry::send("CAN", "CAN_SNIFFER", "CanSniffer", "AKTIV", "", "OK");
            SenderTelemetry::send("CAN", "CAN_BASELINE", "CanBaseline", "JA", "", "OK");
            SenderTelemetry::send("CAN", "CAN_CANDIDATES", "CanCandidates", "3", "", "OK");
            SenderTelemetry::send("CAN", "CAN_CANDIDATE_ID", "CanCandidateId", "0x3C0 B2", "", "OK");
            SenderTelemetry::send("DTC", "ACTIVE", "DTC",
                                  scenario == Simulation::Scenario::NormalSingleFrame ? "Keine" : "P0133 P0420",
                                  "",
                                  scenario == Simulation::Scenario::NormalSingleFrame ? "OK" : "WARN");
            break;
    }
}
}

namespace SenderSimulationScheduler {

void reset() {
    lastSimulationSendAt = 0;
    simulationSampleIndex = 0;
    simulationStatusIndex = 0;
}

void tick(uint32_t nowMs) {
    if (nowMs - lastSimulationSendAt < SenderConfig::SimulationIntervalMs) return;
    lastSimulationSendAt = nowMs;

    const auto scenario = Simulation::RuntimeSimulation::scenario();
    const size_t simulatedPidCount = Simulation::simulatedPidCount();
    const size_t valuesPerTick = SenderConfig::SimulationValuesPerTick == 0 ? 1 : SenderConfig::SimulationValuesPerTick;

    for (size_t index = 0; index < valuesPerTick; ++index) {
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
        if (simulatedPidCount > 0) {
            simulationSampleIndex %= simulatedPidCount;
        }
    }

    // Status frames are sent as a round-robin instead of one large burst. This
    // keeps ESP-NOW traffic smoother and prevents display values from timing out
    // while the simulation walks through all OBD values.
    const size_t statusFramesPerTick = SenderConfig::SimulationStatusFramesPerTick == 0 ? 1 : SenderConfig::SimulationStatusFramesPerTick;
    for (size_t index = 0; index < statusFramesPerTick; ++index) {
        sendSimulationStatusFrame(simulationStatusIndex++, scenario);
    }
}

} // namespace SenderSimulationScheduler
