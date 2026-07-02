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

// region ========================== Laufzeitvariablen ==========================
uint32_t currentMillis;
uint32_t last_can_msg_timestamp = 0;
uint32_t last_heartbeat_send_time = 0;
uint32_t last_twai_status_log_time = 0;
uint32_t heartbeat_count = 0;
bool can_bus_active = false;
bool can_driver_ready = false;
bool esp_now_ready = false;

// End region ========================== Laufzeitvariablen ==========================

// region ========================== Funktionsprototypen ==========================
void sendHeartbeat();
void updateWebConsoleStatus();
// End region ========================== Funktionsprototypen ==========================

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

  last_can_msg_timestamp = millis() - kCanIdleTimeoutMs + 5;

  const bool espNowReady = SenderEspNow::begin();
  esp_now_ready = espNowReady;
  can_driver_ready = Simulation::RuntimeSimulation::enabled() || CANHandler::init();

  WebConsoleHandler::begin();
  OTAHandler::initOTA();

  if (!espNowReady) {
    SenderTelemetry::setLastError("ESP-NOW Initialisierung fehlgeschlagen");
    WebConsoleHandler::log("[Sender] ESP-NOW Init fehlgeschlagen, WebConsole/OTA bleiben aktiv");
  }

  if (!can_driver_ready) {
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
    SenderTelemetry::sendStatus("CAN", can_driver_ready ? "READY" : "INIT_FAIL", can_driver_ready ? "OK" : "ERROR");
    sendHeartbeat();
  }

  WebConsoleHandler::log(can_driver_ready ? "Sender bereit" : "Sender im Fehler-/OTA-Modus");
  SenderLedButton::pulseError(millis());
}
// End region ================================== Setup ==================================

void sendHeartbeat() {
  const uint32_t now = millis();
  SenderHeartbeat::Input input{};
  input.espNowReady = esp_now_ready;
  input.senderRunning = WebConsoleHandler::isStarted();
  input.canDriverReady = can_driver_ready;
  input.canRecent = last_can_msg_timestamp > 0 &&
                    now - last_can_msg_timestamp <= DisplayConfig::CanTimeoutMs;
  input.obdEnabled = SenderConfig::EnableOBD2;
  input.obdRecent = SenderTelemetry::lastObdResponseAt() > 0 &&
                    now - SenderTelemetry::lastObdResponseAt() <= DisplayConfig::ObdTimeoutMs;
  input.udsEnabled = SenderConfig::EnableUDS;
  input.udsAvailable = Uds::Diagnostics::available();
  input.simulationEnabled = Simulation::RuntimeSimulation::enabled();

  SenderHeartbeat::tick(now, last_heartbeat_send_time, heartbeat_count, input, SenderTelemetry::sendStatus);
}

void updateWebConsoleStatus() {
  SenderWebStatus::Input input;
  input.canBusActive = can_bus_active;
  input.canDriverReady = can_driver_ready;
  input.espNowReady = esp_now_ready;
  input.lastCanMessageAt = last_can_msg_timestamp;
  input.heartbeatCount = heartbeat_count;
  WebConsoleHandler::updateRuntimeStatus(SenderWebStatus::build(input));
}

// end region ================ PRINT_CAN_FLAG ================

// region ================================== loop ==================================
void SenderApp::tick() {
  currentMillis = millis();
  OTAHandler::handleOTA();
  WebConsoleHandler::handle();
  updateWebConsoleStatus();
  SenderLedButton::updateLedTestButton();
  sendHeartbeat();

  if (Simulation::RuntimeSimulation::enabled()) {
    SenderSimulationScheduler::tick(currentMillis);
    return;
  }

  if (!WebConsoleHandler::isStarted()) {
    return;
  }

  if (!can_driver_ready) {
    return;
  }

  const SenderCanAlerts::Result canAlerts = SenderCanAlerts::process(kPollingRateMs, SenderTelemetry::sendStatus);

  //------------- CAN Empfang -------------
  if (canAlerts.rxData) {
    last_can_msg_timestamp = currentMillis;
  }
  //---------------------------------------

  //------------ Fehleranzeigen ------------
  if (canAlerts.errorText.length() > 0) {
    SenderTelemetry::setLastError(canAlerts.errorText);
  }
  //---------------------------------------

  //------------- LED Steuerung ------------
      // blink LED if needed
  if (canAlerts.errorLedRequested) {
    SenderLedButton::pulseError(currentMillis);
  }

  SenderLedButton::update(currentMillis);
  //---------------------------------------

  //------------- OBD2 Daten regelmäßig abfragen -------------
  SenderObdScheduler::tick(currentMillis,
                           last_can_msg_timestamp,
                           SenderTelemetry::lastObdResponseAtRef(),
                           can_bus_active,
                           SenderTelemetry::send,
                           SenderTelemetry::sendStatus);

  SenderUdsScheduler::tick(currentMillis,
                           last_can_msg_timestamp,
                           SenderTelemetry::lastObdResponseAtRef(),
                           can_bus_active,
                           SenderTelemetry::send,
                           SenderTelemetry::sendStatus);
  //----------------------------------------------------------

  //--------------TWAI Status-----------
  // Optional zur Fehlersuche
  if (LoggingConfig::TwaiDebugEnabled &&
      currentMillis - last_twai_status_log_time >= SenderConfig::TwaiStatusLogIntervalMs) {
    CANHandler::printStatus(); // Aktivieren / Deaktivieren: #define TWAI_DEBUG_FLAG 0
    last_twai_status_log_time = currentMillis;
  }
  //-------------------------------------------------------

  SenderPowerScheduler::tick(currentMillis);

  //------------Reduziert Leistung des ESP-------------
  //SenderPower::reduceHeat(); // Optional
  //-------------------------------------------------------
}
// End region ================================== loop ==================================
