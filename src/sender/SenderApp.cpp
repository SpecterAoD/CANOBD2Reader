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
#if __has_include("TelemetryProtocol.h")
#include "TelemetryProtocol.h"
#else
  #include "../include/TelemetryProtocol.h"
#endif
#include "TelemetryCodec.h"
#include "TelemetrySequence.h"
#include "RuntimeSimulation.h"
#include "ObdSimulation.h"
#include "IsoTpSimulation.h"
#if __has_include("SimulationData.h")
  #include "SimulationData.h"
#else
  #include "../include/SimulationData.h"
#endif
#include "CANHandler.h"
#include "OTAHandler.h"
#include "SenderPower.h"
#include "WebConsoleHandler.h"
#include "DiagnosticLog.h"
#include "EspNowTelemetryTransport.h"
#include "ObdDiagnostics.h"
#include "UdsDiagnostics.h"
#include "SenderEspNow.h"
#include "SenderLedButton.h"
#include "SenderHeartbeat.h"
#include "SenderCanAlerts.h"
#include "SenderObdScheduler.h"
#include "SenderPowerScheduler.h"
#include "SenderUdsScheduler.h"
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
constexpr size_t kMaxPayloadLength = ProjectConfig::TelemetryPayloadSize;
constexpr uint8_t kVoltageDividerPin = SenderConfig::VoltageDividerPin;
constexpr uint32_t kPollingRateMs = SenderConfig::PollingRateMs;
constexpr uint32_t kCanIdleTimeoutMs = SenderConfig::CanIdleTimeoutMs;
constexpr uint32_t kSimulationSendIntervalMs = SenderConfig::SimulationIntervalMs;
}

// region ================================== struct ==================================
using EspNowTelemetryFrame = Telemetry::TelemetryPacket;

// End region ================================== struct ==================================

// region ========================== Laufzeitvariablen ==========================
uint32_t currentMillis;
uint32_t last_can_msg_timestamp = 0;
uint32_t last_heartbeat_send_time = 0;
uint32_t last_obd_response_time = 0;
uint32_t last_simulation_send_time = 0;
uint32_t last_twai_status_log_time = 0;
uint32_t telemetry_sequence = 0;
uint32_t telemetry_send_ok = 0;
uint32_t telemetry_send_fail = 0;
uint32_t telemetry_last_summary_ms = 0;
uint32_t heartbeat_count = 0;
size_t simulation_sample_index = 0;
bool can_bus_active = false;
bool can_driver_ready = false;
bool esp_now_ready = false;
String last_error_text = "";
String last_send_error_text = "";

// End region ========================== Laufzeitvariablen ==========================

// region ========================== Funktionsprototypen ==========================
void sendTelemetry(const char* type, const char* key, const char* name, const char* value, const char* unit, const char* status);
void sendStatusFrame(const char* key, const char* value, const char* status);
void sendHeartbeat();
void updateWebConsoleStatus();
void maybeLogTelemetrySendSummary();
void sendSimulationTelemetry();
// End region ========================== Funktionsprototypen ==========================

// region ================================== ESP-NOW ==================================
EspNowTelemetryFrame text_frame;
char last_telemetry_payload[kMaxPayloadLength] = {};
// End region ================================== ESP-NOW ==================================

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
    last_error_text = "ESP-NOW Initialisierung fehlgeschlagen";
    WebConsoleHandler::log("[Sender] ESP-NOW Init fehlgeschlagen, WebConsole/OTA bleiben aktiv");
  }

  if (!can_driver_ready) {
    last_error_text = "CAN Initialisierung fehlgeschlagen";
    WebConsoleHandler::log("[Sender] CAN Init fehlgeschlagen, WebConsole/OTA bleiben aktiv");
  }

  WebConsoleHandler::log(SenderConfig::RequireWebStart
                           ? "[SENDER] Manual web start required"
                           : "[SENDER] Auto start enabled");

  if (espNowReady) {
    sendTelemetry("STATUS", "FW", "Firmware", ProjectConfig::FirmwareVersion, "", "OK");
    char protocolVersion[4];
    snprintf(protocolVersion, sizeof(protocolVersion), "%u", ProjectConfig::ProtocolVersion);
    sendTelemetry("STATUS", "PROTO", "Protocol", protocolVersion, "", "OK");
    sendStatusFrame("CAN", can_driver_ready ? "READY" : "INIT_FAIL", can_driver_ready ? "OK" : "ERROR");
    sendHeartbeat();
  }

  WebConsoleHandler::log(can_driver_ready ? "Sender bereit" : "Sender im Fehler-/OTA-Modus");
  SenderLedButton::pulseError(millis());
}
// End region ================================== Setup ==================================

// region ================ Telemetrie senden ================
void sendTelemetry(const char* type,
                   const char* key,
                   const char* name,
                   const char* value,
                   const char* unit,
                   const char* status) {
  TelemetryProtocol::buildPayload(last_telemetry_payload, sizeof(last_telemetry_payload),
                                  type, key, name, value, unit, status,
                                  telemetry_sequence = Telemetry::nextSequence());

  const Telemetry::PacketType packetType =
      strcmp(type, "OBD") == 0 ? Telemetry::PacketType::Obd :
      strcmp(type, "CAN") == 0 ? Telemetry::PacketType::Can :
      strcmp(type, "DTC") == 0 ? Telemetry::PacketType::Diagnostic :
      strcmp(type, "STATUS") == 0 ? Telemetry::PacketType::Status :
      Telemetry::PacketType::Text;

  Telemetry::TelemetryCodec::encodeText(text_frame,
                                        packetType,
                                        telemetry_sequence,
                                        millis(),
                                        last_telemetry_payload);
  const esp_err_t sendResult = Transport::sendTelemetryPacket(text_frame);
  if (sendResult == ESP_OK) {
    ++telemetry_send_ok;
    last_send_error_text = "";
    if (strcmp(type, "STATUS") == 0 && strcmp(key, "HEARTBEAT") == 0) {
      WebConsoleHandler::log("[ESP-NOW] Heartbeat sent seq=" + String(static_cast<unsigned long>(telemetry_sequence)));
    }
  } else {
    ++telemetry_send_fail;
    last_send_error_text = "ESP-NOW send error " + String(static_cast<int>(sendResult));
    last_error_text = last_send_error_text;
    WebConsoleHandler::log("[ESP-NOW] Send failed error=" + String(static_cast<int>(sendResult)) +
                           " seq=" + String(static_cast<unsigned long>(telemetry_sequence)));
    if (LoggingConfig::TraceSenderTelemetry && LoggingConfig::SerialEnabled) {
      WebConsoleHandler::log("[sender-tx] send failed seq=" +
                             String(static_cast<unsigned long>(telemetry_sequence)) +
                             " err=" + String(static_cast<int>(sendResult)));
    }
  }

  maybeLogTelemetrySendSummary();
  WebConsoleHandler::recordTelemetry(last_telemetry_payload);
  if (strcmp(status, "OK") != 0) {
    last_error_text = String(type) + "/" + key + ": " + status;
  }
  if (strcmp(type, "OBD") == 0 && strcmp(status, "OK") == 0) {
    last_obd_response_time = millis();
  }
}

void maybeLogTelemetrySendSummary() {
  if (!(LoggingConfig::TraceSenderTelemetry && LoggingConfig::SerialEnabled)) return;

  const uint32_t now = millis();
  if (telemetry_last_summary_ms == 0) {
    telemetry_last_summary_ms = now;
    return;
  }

  if (now - telemetry_last_summary_ms < LoggingConfig::TraceSummaryIntervalMs) return;

  WebConsoleHandler::log("[sender-tx] seq=" + String(static_cast<unsigned long>(telemetry_sequence)) +
                         " ok=" + String(static_cast<unsigned long>(telemetry_send_ok)) +
                         " fail=" + String(static_cast<unsigned long>(telemetry_send_fail)) +
                         " sim=" + String(Simulation::RuntimeSimulation::enabled() ? "on" : "off"));
  telemetry_last_summary_ms = now;
}

void sendStatusFrame(const char* key, const char* value, const char* status) {
  sendTelemetry("STATUS", key, key, value, "", status);
}

void sendHeartbeat() {
  const uint32_t now = millis();
  SenderHeartbeat::Input input{};
  input.espNowReady = esp_now_ready;
  input.senderRunning = WebConsoleHandler::isStarted();
  input.canDriverReady = can_driver_ready;
  input.canRecent = last_can_msg_timestamp > 0 &&
                    now - last_can_msg_timestamp <= DisplayConfig::CanTimeoutMs;
  input.obdEnabled = SenderConfig::EnableOBD2;
  input.obdRecent = last_obd_response_time > 0 &&
                    now - last_obd_response_time <= DisplayConfig::ObdTimeoutMs;
  input.udsEnabled = SenderConfig::EnableUDS;
  input.udsAvailable = Uds::Diagnostics::available();
  input.simulationEnabled = Simulation::RuntimeSimulation::enabled();

  SenderHeartbeat::tick(now, last_heartbeat_send_time, heartbeat_count, input, sendStatusFrame);
}

void sendSimulationTelemetry() {
  if (millis() - last_simulation_send_time < kSimulationSendIntervalMs) return;
  last_simulation_send_time = millis();

  const auto scenario = Simulation::RuntimeSimulation::scenario();
  const auto simulated = Simulation::simulatedPidValue(simulation_sample_index, millis(), scenario);
  char value[16];
  snprintf(value, sizeof(value), simulated.decimals == 0 ? "%.0f" : "%.1f", static_cast<double>(simulated.value));
  sendTelemetry(simulated.type,
                simulated.key,
                simulated.name,
                strcmp(simulated.status, "OK") == 0 ? value : "N/A",
                simulated.unit,
                simulated.status);

  simulation_sample_index++;

  // Status-, CAN- und DTC-Pakete werden zwischen die Messwerte gemischt, damit
  // alle Display-Seiten ohne Fahrzeug und ohne CAN-Transceiver getestet werden koennen.
  if (simulation_sample_index % Simulation::simulatedPidCount() == 0) {
    const auto isoTp = Simulation::buildIsoTpSequence(scenario);
    sendStatusFrame("CAN", "SIMULATED", "OK");
    sendStatusFrame("OBD", isoTp.timeoutExpected ? "TIMEOUT" : "SIMULATED", isoTp.timeoutExpected ? "TIMEOUT" : "OK");
    sendStatusFrame("SIM", Simulation::RuntimeSimulation::enabled() ? "ACTIVE" : "INACTIVE", "OK");
    sendTelemetry("STATUS", "SIM_SCENARIO", "SimScenario", Simulation::RuntimeSimulation::scenarioName(), "", "OK");
    sendTelemetry("STATUS", "SIM_DETAIL", "SimDetail", Simulation::scenarioDiagnosticText(scenario), "", "OK");
    sendTelemetry("CAN", "RAW", "LastCAN", "0x7E8 DLC8 04 41 0C 1A F8 55 55 55", "", "OK");
    sendTelemetry("CAN", "HINT", "CANHint", Simulation::scenarioDiagnosticText(scenario), "", isoTp.negativeResponse ? "ERROR" : (isoTp.timeoutExpected ? "TIMEOUT" : "OK"));
    sendTelemetry("CAN", "COUNT", "CANCount", "128", "frames", "OK");
    sendTelemetry("DTC", "ACTIVE", "DTC",
                  scenario == Simulation::Scenario::NormalSingleFrame ? "Keine" : "P0133 P0420",
                  "",
                  scenario == Simulation::Scenario::NormalSingleFrame ? "OK" : "WARN");
  }
}
// end region ================ Telemetrie senden ================

void updateWebConsoleStatus() {
  Runtime::WebRuntimeStatus status;
  status.canActive = can_bus_active;
  status.obdActive = SenderConfig::EnableOBD2 && last_obd_response_time > 0 &&
                     millis() - last_obd_response_time <= DisplayConfig::ObdTimeoutMs;
  status.pidSupportReady = SenderObdScheduler::supportedPidsInitialized();
  status.simulationActive = Simulation::RuntimeSimulation::enabled();
  status.simulationScenario = Simulation::RuntimeSimulation::scenarioName();
  status.batteryVoltage = SenderPower::getLastVoltage();
  status.uptimeMs = millis();
  status.telemetrySequence = telemetry_sequence;
  status.telemetrySendOk = telemetry_send_ok;
  status.telemetrySendFail = telemetry_send_fail;
  status.heartbeatCount = heartbeat_count;
  status.lastCanAgeMs = last_can_msg_timestamp == 0 ? 0 : millis() - last_can_msg_timestamp;
  status.lastObdAgeMs = last_obd_response_time == 0 ? 0 : millis() - last_obd_response_time;
  status.canState = can_driver_ready ? (can_bus_active ? "ACTIVE" : "IDLE") : "INIT_FAIL";
  status.obdState = SenderConfig::EnableOBD2 ? (status.obdActive ? "ACTIVE" : "NO_RESPONSE") : "DISABLED";
  status.espNowState = esp_now_ready ? "READY" : "INIT_FAIL";
  status.lastSendError = last_send_error_text;
  status.lastDtc = SenderObdScheduler::lastDtcText();
  status.lastVin = SenderObdScheduler::lastVinText();
  status.lastTelemetry = String(last_telemetry_payload);
  status.lastError = last_error_text;
  status.obdRequestCount = Obd::Diagnostics::requestCount();
  status.obdSendFailureCount = Obd::Diagnostics::sendFailureCount();
  status.obdTimeoutCount = Obd::Diagnostics::timeoutCount();
  status.obdValidResponseCount = Obd::Diagnostics::validResponseCount();
  status.obdNegativeResponseCount = Obd::Diagnostics::negativeResponseCount();
  status.obdTimeoutStreak = Obd::Diagnostics::timeoutStreak();
  status.obdPhysicalFallbackActive = Obd::Diagnostics::physicalFallbackActive();
  status.obdRequestCanId = Obd::Diagnostics::requestCanId();
  status.supportedPidMask01_20 = Obd::Diagnostics::supportedPidMask(0);
  status.supportedPidMask21_40 = Obd::Diagnostics::supportedPidMask(1);
  status.supportedPidMask41_60 = Obd::Diagnostics::supportedPidMask(2);
  status.lastObdRequest = Obd::Diagnostics::lastRequest();
  status.lastEcuResponse = Obd::Diagnostics::lastEcuResponse();
  status.lastNegativeResponse = Obd::Diagnostics::lastNegativeResponse();
  status.udsAvailable = Uds::Diagnostics::available();
  status.udsRequestCount = Uds::Diagnostics::requestCount();
  status.udsSendFailureCount = Uds::Diagnostics::sendFailureCount();
  status.udsTimeoutCount = Uds::Diagnostics::timeoutCount();
  status.udsPositiveResponseCount = Uds::Diagnostics::positiveResponseCount();
  status.udsNegativeResponseCount = Uds::Diagnostics::negativeResponseCount();
  status.lastUdsRequest = Uds::Diagnostics::lastRequest();
  status.lastUdsResponse = Uds::Diagnostics::lastResponse();
  status.lastUdsNegativeResponse = Uds::Diagnostics::lastNegativeResponse();
  status.lastUdsDid = SenderUdsScheduler::lastDidText();
  status.lastUdsDtc = SenderUdsScheduler::lastDtcText();
  WebConsoleHandler::updateRuntimeStatus(status);
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
    sendSimulationTelemetry();
    return;
  }

  if (!WebConsoleHandler::isStarted()) {
    return;
  }

  if (!can_driver_ready) {
    return;
  }

  const SenderCanAlerts::Result canAlerts = SenderCanAlerts::process(kPollingRateMs, sendStatusFrame);

  //------------- CAN Empfang -------------
  if (canAlerts.rxData) {
    last_can_msg_timestamp = currentMillis;
  }
  //---------------------------------------

  //------------ Fehleranzeigen ------------
  if (canAlerts.errorText.length() > 0) {
    last_error_text = canAlerts.errorText;
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
                           last_obd_response_time,
                           can_bus_active,
                           sendTelemetry,
                           sendStatusFrame);

  SenderUdsScheduler::tick(currentMillis,
                           last_can_msg_timestamp,
                           last_obd_response_time,
                           can_bus_active,
                           sendTelemetry,
                           sendStatusFrame);
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
