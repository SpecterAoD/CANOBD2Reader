/*============================================================================
	 CAN_OBD2_Gateway
	 Version: 0.0.1 
	 
	 Dieses Projekt basiert auf dem CAN-Bus Gateway von MrDIY.ca.
   Die Originalversion wurde als Grundlage verwendet und anschließend an die 
   spezifischen Anforderungen des Projekts angepasst.

   Ursprüngliches Repository:
   MrDIY CAN-Bus Gateway https://gitlab.com/MrDIYca/canabus

============================================================================= */

// region ================================== Includes ==================================
#include <Arduino.h>
#include "SenderApp.h"
#include <string.h>
#include "common_config.h"
#include "config/ProjectConfig.h"
#include "config/SenderConfig.h"
#include "config/DisplayConfig.h"
#include "config/LoggingConfig.h"
#include "RuntimeSimulation.h"
#include "CANHandler.h"
#include "OTAHandler.h"
#include "SenderPower.h"
#include "WebConsoleHandler.h"
#include "DiagnosticLog.h"
#include "ObdDiagnostics.h"
#include "UdsDiagnostics.h"
#include "SenderEspNow.h"
#include "SenderLedButton.h"
#include "SenderHeartbeat.h"
#include "SenderCanAlerts.h"
#include "SenderObdScheduler.h"
#include "SenderPowerScheduler.h"
#include "SenderSimulationScheduler.h"
#include "SenderTelemetry.h"
#include "SenderUdsScheduler.h"
#include "SenderWebStatus.h"
#include "SenderLoopState.h"
#include "SenderRuntimeCoordinator.h"
// End region ================================== Includes ==================================

// region ================================== #define ==================================
// --------- Firmware ---------
#define FIRMWARE_VERSION CANOBD2_FIRMWARE_VERSION
// -----------------------------

// ------ Debug Optionen -------
#define debug(...) do { if (LoggingConfig::GeneralDebugEnabled && LoggingConfig::SerialEnabled) { Serial.print(__VA_ARGS__); } } while (0)
#define debugln(...) do { if (LoggingConfig::GeneralDebugEnabled && LoggingConfig::SerialEnabled) { Serial.println(__VA_ARGS__); } } while (0)
#define power_debug(...) do { if (LoggingConfig::PowerDebugEnabled && LoggingConfig::SerialEnabled) { Serial.print(__VA_ARGS__); } } while (0)
#define power_debugln(...) do { if (LoggingConfig::PowerDebugEnabled && LoggingConfig::SerialEnabled) { Serial.println(__VA_ARGS__); } } while (0)
// -----------------------------

// End region ================================== #define ==================================

namespace {
constexpr uint8_t kVoltageDividerPin = SenderConfig::VoltageDividerPin;
constexpr uint32_t kPollingRateMs = SenderConfig::PollingRateMs;
constexpr uint32_t kCanIdleTimeoutMs = SenderConfig::CanIdleTimeoutMs;
}

namespace {

void sendHeartbeat(Runtime::SenderLoopState& runtimeState) {
  const uint32_t now = millis();
  SenderHeartbeat::Input input{};
  input.espNowReady = runtimeState.espNowReady;
  input.senderRunning = WebConsoleHandler::isStarted();
  input.canDriverReady = runtimeState.canDriverReady;
  input.canRecent = runtimeState.canRecent(now, DisplayConfig::CanTimeoutMs);
  input.obdEnabled = SenderConfig::EnableOBD2;
  input.obdRecent = SenderTelemetry::lastObdResponseAt() > 0 &&
                    now - SenderTelemetry::lastObdResponseAt() <= DisplayConfig::ObdTimeoutMs;
  input.udsEnabled = SenderConfig::EnableUDS;
  input.udsAvailable = Uds::Diagnostics::available();
  input.simulationEnabled = Simulation::RuntimeSimulation::enabled();

  SenderHeartbeat::tick(now,
                        runtimeState.lastHeartbeatSentAt,
                        runtimeState.heartbeatCount,
                        input,
                        SenderTelemetry::sendStatus);
}

void updateWebConsoleStatus(const Runtime::SenderLoopState& runtimeState) {
  SenderWebStatus::Input input;
  input.canBusActive = runtimeState.canBusActive;
  input.canDriverReady = runtimeState.canDriverReady;
  input.espNowReady = runtimeState.espNowReady;
  input.lastCanMessageAt = runtimeState.lastCanMessageAt;
  input.heartbeatCount = runtimeState.heartbeatCount;
  WebConsoleHandler::updateRuntimeStatus(SenderWebStatus::build(input));
}

void updateLedTestButton() {
  // The button module returns whether the LED test is currently active.
  // The runtime coordinator only needs to trigger the non-blocking update.
  (void)SenderLedButton::updateLedTestButton();
}

Runtime::SenderRuntimeCoordinator::CanAlertResult processCanAlerts(uint32_t waitMs) {
  const SenderCanAlerts::Result result = SenderCanAlerts::process(waitMs, SenderTelemetry::sendStatus);
  static String lastCanError;
  lastCanError = result.errorText;
  return {result.rxData, result.errorLedRequested, lastCanError.c_str()};
}

void tickObd(Runtime::SenderLoopState& runtimeState) {
  SenderObdScheduler::tick(runtimeState.currentMillis,
                           runtimeState.lastCanMessageAt,
                           SenderTelemetry::lastObdResponseAtRef(),
                           runtimeState.canBusActive,
                           SenderTelemetry::send,
                           SenderTelemetry::sendStatus);
}

void tickUds(Runtime::SenderLoopState& runtimeState) {
  SenderUdsScheduler::tick(runtimeState.currentMillis,
                           runtimeState.lastCanMessageAt,
                           SenderTelemetry::lastObdResponseAtRef(),
                           runtimeState.canBusActive,
                           SenderTelemetry::send,
                           SenderTelemetry::sendStatus);
}

void setLastTelemetryError(const char* errorText) {
  SenderTelemetry::setLastError(String(errorText == nullptr ? "" : errorText));
}

Runtime::SenderRuntimeCoordinator::Services makeCoordinatorServices() {
  Runtime::SenderRuntimeCoordinator::Services services{};
  services.handleOta = OTAHandler::handleOTA;
  services.handleWeb = WebConsoleHandler::handle;
  services.updateWebStatus = updateWebConsoleStatus;
  services.updateLedTestButton = updateLedTestButton;
  services.sendHeartbeat = sendHeartbeat;
  services.simulationEnabled = Simulation::RuntimeSimulation::enabled;
  services.tickSimulation = SenderSimulationScheduler::tick;
  services.senderStarted = WebConsoleHandler::isStarted;
  services.processCanAlerts = processCanAlerts;
  services.setLastError = setLastTelemetryError;
  services.pulseErrorLed = SenderLedButton::pulseError;
  services.updateLed = SenderLedButton::update;
  services.tickObd = tickObd;
  services.tickUds = tickUds;
  services.logTwaiStatus = CANHandler::printStatus;
  services.tickPower = SenderPowerScheduler::tick;
  return services;
}

Runtime::SenderRuntimeCoordinator coordinator(
    {kPollingRateMs, SenderConfig::TwaiStatusLogIntervalMs, LoggingConfig::TwaiDebugEnabled},
    makeCoordinatorServices());

} // namespace

// region ================================== Setup ==================================
void SenderApp::begin() {
  Simulation::RuntimeSimulation::resetForBoot();
  Serial.begin(ProjectConfig::Baudrate);
  debugln(ProjectConfig::Baudrate);
  DiagnosticLog::begin();
  DiagnosticLog::appendf("[BOOT] Sender firmware=%s protocol=%u target=%s",
                         ProjectConfig::FirmwareVersion,
                         ProjectConfig::ProtocolVersion,
                         ProjectConfig::TargetName);
  Obd::Diagnostics::reset();
  Uds::Diagnostics::reset();
  SenderObdScheduler::reset();
  SenderPowerScheduler::reset();
  SenderSimulationScheduler::reset();
  SenderTelemetry::reset();
  SenderUdsScheduler::reset();

  SenderLedButton::begin();
  pinMode(kVoltageDividerPin, INPUT);

  debugln("\n------------------------");
  debugln("         SpecterAoD");
  debugln("       CAN OBD2 SHIELD");
  debugln("      based on MrDiy.ca");
  debugln("https://gitlab.com/MrDIYca/canabus");
  debugln("------------------------");

  const bool espNowReady = SenderEspNow::begin();
  const bool canDriverReady = Simulation::RuntimeSimulation::enabled() || CANHandler::init();
  coordinator.resetForBoot(millis(), kCanIdleTimeoutMs, espNowReady, canDriverReady);

  WebConsoleHandler::begin();
  OTAHandler::initOTA();

  if (!espNowReady) {
    SenderTelemetry::setLastError("ESP-NOW Initialisierung fehlgeschlagen");
    WebConsoleHandler::log("[Sender] ESP-NOW Init fehlgeschlagen, WebConsole/OTA bleiben aktiv");
  }

  if (!canDriverReady) {
    SenderTelemetry::setLastError("CAN Initialisierung fehlgeschlagen");
    WebConsoleHandler::log("[Sender] CAN Init fehlgeschlagen, WebConsole/OTA bleiben aktiv");
  }

  WebConsoleHandler::log(SenderConfig::RequireWebStart
                           ? "[SENDER] Manual web start required"
                           : "[SENDER] Auto start enabled");

  if (espNowReady) {
    SenderTelemetry::send("STATUS", "FW", "Firmware", ProjectConfig::FirmwareVersion, "", "OK");
    char protocolVersion[4];
    snprintf(protocolVersion, sizeof(protocolVersion), "%u", ProjectConfig::ProtocolVersion);
    SenderTelemetry::send("STATUS", "PROTO", "Protocol", protocolVersion, "", "OK");
    SenderTelemetry::sendStatus("CAN",
                                canDriverReady ? "READY" : "INIT_FAIL",
                                canDriverReady ? "OK" : "ERROR");
    sendHeartbeat(coordinator.state());
  }

  WebConsoleHandler::log(canDriverReady ? "Sender bereit" : "Sender im Fehler-/OTA-Modus");
  SenderLedButton::pulseError(millis());
}
// End region ================================== Setup ==================================

// end region ================ PRINT_CAN_FLAG ================

// region ================================== loop ==================================
void SenderApp::tick() {
  coordinator.tick(millis());
}
// End region ================================== loop ==================================
