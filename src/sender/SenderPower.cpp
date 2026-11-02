#include "SenderPower.h"
#include "UpdateSafety.h"

#include <Arduino.h>

#include "RuntimeSimulation.h"
#include "SenderRuntimeState.h"
#include "SenderTelemetry.h"
#include "SenderLiveData.h"
#include "WebConsoleHandler.h"
#include "config/PowerConfig.h"
#include "config/SenderConfig.h"

namespace {
Power::ActivityMonitor activityMonitor;
Power::ActivitySnapshot lastSnapshot;
uint32_t bootCompletedAtMs = 0;
float lastVoltage = 0.0f;

struct VoltageMeasurement {
  int adcRaw = 0;
  float voltage = 0.0f;
};

VoltageMeasurement readVinVoltage() {
  VoltageMeasurement measurement;
  measurement.adcRaw = analogRead(SenderConfig::VoltageDividerPin);
  const float adcVoltage = measurement.adcRaw * (3.3f / 4095.0f);
  measurement.voltage = adcVoltage * SenderConfig::VoltageCalcFactor;
  lastVoltage = measurement.voltage;
  return measurement;
}

void logStateChange(Power::VehicleState previousState, Power::VehicleState nextState) {
  if (previousState == nextState) return;
  WebConsoleHandler::log(String("[POWER] vehicleState=") +
                         Power::ActivityMonitor::vehicleStateName(nextState));
}

void sendPowerStatus(const Power::ActivitySnapshot& snapshot) {
  char scoreText[8];
  snprintf(scoreText, sizeof(scoreText), "%u", static_cast<unsigned int>(snapshot.activityScore));

  SenderTelemetry::sendStatus("POWER_STATE",
                              Power::ActivityMonitor::vehicleStateName(snapshot.state),
                              "OK");
  SenderTelemetry::sendStatus("ACTIVITY_SCORE", scoreText, "OK");
  SenderTelemetry::sendStatus("POWER_COMMAND",
                              Power::ActivityMonitor::powerCommandName(snapshot.command),
                              snapshot.command == Power::PowerCommand::None ? "OK" : "WARN");
}
}

namespace SenderPower {
  void reduceHeat() {
    setCpuFrequencyMhz(SenderConfig::CpuFrequency);
  }

  void updateActivity(const Runtime::SenderLoopState& state, uint32_t lastObdResponseAt) {
    if (bootCompletedAtMs == 0) {
      bootCompletedAtMs = state.currentMillis;
      activityMonitor.reset(state.currentMillis);
    }

    const VoltageMeasurement measurement = readVinVoltage();

    Power::ActivityInput input;
    input.nowMs = state.currentMillis;
    input.bootCompletedAtMs = bootCompletedAtMs;
    input.lastCanMessageAtMs = state.lastCanMessageAt;
    input.lastObdResponseAtMs = lastObdResponseAt;
    input.lastUserActivityAtMs = 0;
    input.rpm = Runtime::SenderRuntimeState::lastRpm();
    input.speedKph = Runtime::SenderRuntimeState::lastSpeedKph();
    input.batteryVoltage = measurement.voltage;
    input.engineLoadPercent = Runtime::SenderRuntimeState::lastEngineLoadPercent();
    input.throttlePercent = Runtime::SenderRuntimeState::lastThrottlePercent();
    input.simulationActive = Simulation::RuntimeSimulation::enabled();
    input.canDriverReady = state.canDriverReady;
    input.obdEnabled = SenderConfig::EnableOBD2;

    const Power::VehicleState previousState = lastSnapshot.state;
    lastSnapshot = activityMonitor.update(input);
    logStateChange(previousState, lastSnapshot.state);
    FirmwareUpdate::updateSafety.observe(
        Simulation::RuntimeSimulation::enabled() ? FirmwareUpdate::VehicleCondition::Unknown
            : FirmwareUpdate::vehicleCondition(Power::ActivityMonitor::vehicleStateName(lastSnapshot.state)),
        state.currentMillis);

    if (lastSnapshot.command != Power::PowerCommand::None) {
      WebConsoleHandler::log(String("[POWER] command=") +
                             Power::ActivityMonitor::powerCommandName(lastSnapshot.command));
      // The command exists for exactly one update; send it right away instead
      // of waiting for the 1 s status cycle (it was practically never sent).
      sendPowerStatus(lastSnapshot);
    }
  }

  void publishPowerTelemetry() {
    sendPowerStatus(lastSnapshot);
  }

  void sendBatteryVoltage() {
    const VoltageMeasurement measurement = readVinVoltage();
    SenderLiveData::update(LiveData::Metric::BatteryVoltage, measurement.voltage);
  }

  float getLastVoltage() {
    if (lastVoltage <= 0.01f) {
      readVinVoltage();
    }
    return lastVoltage;
  }

  bool isCarRunning() {
    return lastSnapshot.state == Power::VehicleState::Running ||
           lastSnapshot.state == Power::VehicleState::StartStop ||
           lastSnapshot.state == Power::VehicleState::Idle;
  }

  const Power::ActivitySnapshot& activitySnapshot() {
    return lastSnapshot;
  }

  const char* vehicleStateName() {
    return Power::ActivityMonitor::vehicleStateName(lastSnapshot.state);
  }

  const char* powerCommandName() {
    return Power::ActivityMonitor::powerCommandName(lastSnapshot.command);
  }
}
