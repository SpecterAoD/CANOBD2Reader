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
#include <esp_now.h>
#include <WiFi.h>
#include <string.h>
#include "driver/twai.h"
#include "common_config.h"
#include "Config.h"
#if __has_include("PIDs.h")
  #include "PIDs.h"
#else
  #include "../include/PIDs.h"
#endif
#if __has_include("PID_Converter.h")
  #include "PID_Converter.h"
#else
  #include "../include/PID_Converter.h"
#endif
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
#include "OBDHandler.h"
#include "OTAHandler.h"
#include "SenderPower.h"
#include "WebConsoleHandler.h"
#include "StatusLogic.h"
#include "DiagnosticLog.h"
#include "EspNowTelemetryTransport.h"
#include "ObdDiagnostics.h"
#include "DtcDecoder.h"
#include "VinDecoder.h"
#include "UdsClient.h"
#include "UdsDecoder.h"
#include "UdsDiagnostics.h"
// End region ================================== Includes ==================================

// region ================================== #define ==================================
// --------- Firmware ---------
#define FIRMWARE_VERSION CANOBD2_FIRMWARE_VERSION
// -----------------------------

// ------ Debug Optionen -------
#define debug(...) do { if (Config::Debug::General && Config::Debug::Serial) { Serial.print(__VA_ARGS__); } } while (0)
#define debugln(...) do { if (Config::Debug::General && Config::Debug::Serial) { Serial.println(__VA_ARGS__); } } while (0)
#define power_debug(...) do { if (Config::Debug::Power && Config::Debug::Serial) { Serial.print(__VA_ARGS__); } } while (0)
#define power_debugln(...) do { if (Config::Debug::Power && Config::Debug::Serial) { Serial.println(__VA_ARGS__); } } while (0)
// -----------------------------

// End region ================================== #define ==================================

namespace {
constexpr size_t kMaxPayloadLength = ProjectConfig::TelemetryPayloadSize;
constexpr uint8_t kLedPin1 = Config::Sender::LedPin1;
constexpr uint8_t kLedPin2 = Config::Sender::LedPin2;
constexpr uint8_t kButtonPin = Config::Sender::ButtonPin;
constexpr uint8_t kVoltageDividerPin = Config::Sender::VoltageDividerPin;
constexpr uint32_t kPollingRateMs = Config::Sender::PollingRateMs;
constexpr uint32_t kCanIdleTimeoutMs = Config::Sender::CanIdleTimeoutMs;
constexpr uint32_t kBatterySendIntervalMs = Config::Sender::BatterySendIntervalMs;
constexpr uint32_t kHeartbeatIntervalMs = Config::Sender::HeartbeatIntervalMs;
constexpr uint32_t kSupportedPidRefreshIntervalMs = Config::Sender::SupportedPidRefreshIntervalMs;
constexpr uint32_t kDtcQueryIntervalMs = Config::Sender::DtcQueryIntervalMs;
constexpr uint32_t kVinQueryIntervalMs = Config::Sender::VinQueryIntervalMs;
constexpr uint32_t kUdsQueryIntervalMs = Config::Sender::UdsQueryIntervalMs;
constexpr uint32_t kSimulationSendIntervalMs = Config::Sender::SimulationIntervalMs;
constexpr uint16_t kUdsVinDid = 0xF190;
}

// region ================================== struct ==================================
using EspNowTelemetryFrame = Telemetry::TelemetryPacket;

// End region ================================== struct ==================================

// region ========================== Laufzeitvariablen ==========================
uint32_t led_last_on_timestamp = 0;
uint32_t currentMillis;
unsigned long last_can_msg_timestamp = 0;
unsigned long last_obd_request_time = 0;
unsigned long last_battery_send_time = 0;
unsigned long last_heartbeat_send_time = 0;
unsigned long last_obd_response_time = 0;
unsigned long last_supported_pid_refresh_time = 0;
unsigned long last_dtc_query_time = 0;
unsigned long last_vin_query_time = 0;
unsigned long last_uds_query_time = 0;
unsigned long last_simulation_send_time = 0;
unsigned long last_led_test_change_time = 0;
unsigned long last_twai_status_log_time = 0;
uint32_t telemetry_sequence = 0;
uint32_t telemetry_send_ok = 0;
uint32_t telemetry_send_fail = 0;
uint32_t telemetry_last_summary_ms = 0;
uint32_t heartbeat_count = 0;
size_t simulation_sample_index = 0;
bool can_bus_active = false;
bool can_driver_ready = false;
bool esp_now_ready = false;
bool led_test_active = false;
bool supported_pids_initialized = false;
uint32_t supported_pids_01_20 = 0;
uint32_t supported_pids_21_40 = 0;
uint32_t supported_pids_41_60 = 0;
String last_dtc_text = "--";
String last_vin_text = "--";
String last_uds_did_text = "--";
String last_uds_dtc_text = "--";
String last_error_text = "";
String last_send_error_text = "";

// End region ========================== Laufzeitvariablen ==========================

// region ========================== Funktionsprototypen ==========================
int getMeshID();
void sendTelemetry(const char* type, const char* key, const char* name, const char* value, const char* unit, const char* status);
void sendStatusFrame(const char* key, const char* value, const char* status);
void sendHeartbeat();
void updateWebConsoleStatus();
void maybeLogTelemetrySendSummary();
void refreshSupportedPIDs();
bool isPIDSupported(byte pid);
void queryAndSendDTCs();
void queryAndSendVin();
void queryAndSendUdsDiagnostics();
void sendSimulationTelemetry();
void handleLedTestButton();
// End region ========================== Funktionsprototypen ==========================

// region ================================== ESP-NOW ==================================
int esp_now_mesh_id;
uint8_t peerAddress[6] = {};

EspNowTelemetryFrame text_frame;
char last_telemetry_payload[kMaxPayloadLength] = {};
esp_now_peer_info_t peerInfo = {};
Uds::Client udsClient;

bool initESPNow() {

  debugln(" ESPNOW............INIT");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    debugln(" ESPNOW.............FAIL");
    return false;
  }
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerAddress, Config::Network::DisplayPeerMac, sizeof(peerAddress));
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = Config::Network::EspNowChannel;
  peerInfo.encrypt = Config::Network::UseEspNowEncryption;

  if (Config::Network::UseEspNowEncryption) {
    memcpy(peerInfo.lmk, Config::Network::EspNowAesKey, 16);
  }

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    debugln(" ESPNOW.......Peer ......FAIL");
    return false;
  }

  debugln(" ESPNOW.......Peer.......OK");

  esp_now_mesh_id = getMeshID();
  debug(" MESH ID..........");
  debugln(esp_now_mesh_id);
  return true;
}

int getMeshID() {
  uint8_t baseMac[6];
  WiFi.macAddress(baseMac);
  uint32_t uniqueID = 0;
  for (int i = 2; i < 6; i++) {
    uniqueID <<= 8;
    uniqueID |= baseMac[i];
  }
  return uniqueID % 32768;
}
// End region ================================== ESP-NOW ==================================

// region ================================== Setup ==================================
void SenderApp::begin() {
  Simulation::RuntimeSimulation::resetForBoot();
  Serial.begin(Config::Project::Baudrate);
  debugln(Config::Project::Baudrate);
  DiagnosticLog::begin();
  DiagnosticLog::appendf("[BOOT] Sender firmware=%s protocol=%u target=%s",
                         Config::Project::FirmwareVersion,
                         Config::Project::ProtocolVersion,
                         Config::Project::TargetName);
  Obd::Diagnostics::reset();
  Uds::Diagnostics::reset();

  pinMode(kLedPin1, OUTPUT);
  pinMode(kLedPin2, OUTPUT);
  pinMode(kButtonPin, INPUT_PULLUP);  // Taster zieht gegen GND
  pinMode(kVoltageDividerPin, INPUT);

  debugln("\n------------------------");
  debugln("         SpecterAoD");
  debugln("       CAN OBD2 SHIELD");
  debugln("      based on MrDiy.ca");
  debugln("https://gitlab.com/MrDIYca/canabus");
  debugln("------------------------");

  last_can_msg_timestamp = millis() - kCanIdleTimeoutMs + 5;

  const bool espNowReady = initESPNow();
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

  WebConsoleHandler::log(Config::Network::RequireWebStart
                           ? "[SENDER] Manual web start required"
                           : "[SENDER] Auto start enabled");

  if (espNowReady) {
    sendTelemetry("STATUS", "FW", "Firmware", Config::Project::FirmwareVersion, "", "OK");
    char protocolVersion[4];
    snprintf(protocolVersion, sizeof(protocolVersion), "%u", Config::Project::ProtocolVersion);
    sendTelemetry("STATUS", "PROTO", "Protocol", protocolVersion, "", "OK");
    sendStatusFrame("CAN", can_driver_ready ? "READY" : "INIT_FAIL", can_driver_ready ? "OK" : "ERROR");
    sendHeartbeat();
  }

  WebConsoleHandler::log(can_driver_ready ? "Sender bereit" : "Sender im Fehler-/OTA-Modus");
  digitalWrite(kLedPin1, HIGH);
}
// End region ================================== Setup ==================================

// region ================= Supported PIDs =================
uint32_t bytesToMask(const byte* data, byte length) {
  if (length < 4) return 0;
  return (static_cast<uint32_t>(data[0]) << 24) |
         (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) |
         static_cast<uint32_t>(data[3]);
}

bool querySupportedPidRange(byte rangePid, uint32_t& targetMask) {
  byte responseData[8];
  byte responseLen = 0;

  if (!OBD2Handler::sendRequest(read_LiveData, rangePid)) return false;
  if (!OBD2Handler::receiveResponse(read_LiveData, rangePid, responseData, responseLen)) return false;

  last_can_msg_timestamp = millis();
  last_obd_response_time = millis();
  can_bus_active = true;
  targetMask = bytesToMask(responseData, responseLen);
  return targetMask != 0;
}

bool isPIDSupported(byte pid) {
  if (!supported_pids_initialized) return true;

  byte rangeBase = 0x00;
  uint32_t mask = supported_pids_01_20;
  if (pid >= 0x21 && pid <= 0x40) {
    rangeBase = 0x20;
    mask = supported_pids_21_40;
  } else if (pid >= 0x41 && pid <= 0x60) {
    rangeBase = 0x40;
    mask = supported_pids_41_60;
  } else if (pid < 0x01 || pid > 0x20) {
    return false;
  }

  const byte offset = pid - rangeBase;
  if (offset == 0 || offset > 32) return false;
  return (mask & (1UL << (32 - offset))) != 0;
}

void refreshSupportedPIDs() {
  bool ok = querySupportedPidRange(SUPPORTED_PIDS_1_20, supported_pids_01_20);

  if (ok && (supported_pids_01_20 & 0x00000001UL)) {
    ok = querySupportedPidRange(SUPPORTED_PIDS_21_40, supported_pids_21_40);
  }

  if (ok && (supported_pids_21_40 & 0x00000001UL)) {
    ok = querySupportedPidRange(SUPPORTED_PIDS_41_60, supported_pids_41_60);
  }

  supported_pids_initialized = ok;
  Obd::Diagnostics::setSupportedPidMasks(supported_pids_01_20,
                                         supported_pids_21_40,
                                         supported_pids_41_60,
                                         ok);
  if (ok) {
    char masks[112];
    snprintf(masks, sizeof(masks), "[OBD] Supported PIDs 01-20=0x%08lX 21-40=0x%08lX 41-60=0x%08lX",
             static_cast<unsigned long>(supported_pids_01_20),
             static_cast<unsigned long>(supported_pids_21_40),
             static_cast<unsigned long>(supported_pids_41_60));
    WebConsoleHandler::log(masks);
  } else {
    WebConsoleHandler::log("[OBD] Supported PID query failed");
  }
  sendStatusFrame("PID_SUPPORT", ok ? "READY" : "UNKNOWN", ok ? "OK" : "WARN");
}
// End region ================= Supported PIDs =================

void queryAndSendDTCs() {
  const uint8_t payload[] = {read_DTCs};
  IsoTp::Payload response{};

  if (!OBD2Handler::requestPayload(read_DTCs, payload, sizeof(payload), 0xFF, response)) {
    last_dtc_text = "SEND_FAIL";
    if (OBD2Handler::lastResponseWasNegative()) {
      last_dtc_text = Obd::Diagnostics::lastNegativeResponse();
      WebConsoleHandler::log("[DTC] Negative response: " + last_dtc_text);
      sendTelemetry("DTC", "ACTIVE", "DTC", last_dtc_text.c_str(), "", "ERROR");
    } else {
      last_dtc_text = "TIMEOUT";
      WebConsoleHandler::log("[DTC] Timeout waiting for ISO-TP response");
      sendTelemetry("DTC", "ACTIVE", "DTC", "N/A", "", "TIMEOUT");
    }
    Obd::Diagnostics::setDtc(last_dtc_text.c_str());
    return;
  }

  char dtcList[64] = {0};
  last_can_msg_timestamp = millis();
  last_obd_response_time = millis();
  can_bus_active = true;
  const size_t dataOffset = response.length > 0 ? 1 : 0; // skip 0x43 positive response byte
  Obd::decodeDtcList(response.bytes.data() + dataOffset,
                     response.length > dataOffset ? response.length - dataOffset : 0,
                     dtcList,
                     sizeof(dtcList));

  if (dtcList[0] == '\0') {
    last_dtc_text = "Keine";
    Obd::Diagnostics::setDtc("Keine");
    WebConsoleHandler::log("[DTC] No active DTCs");
    sendTelemetry("DTC", "ACTIVE", "DTC", "Keine", "", "OK");
  } else {
    last_dtc_text = dtcList;
    Obd::Diagnostics::setDtc(dtcList);
    WebConsoleHandler::log("[DTC] Active codes: " + String(dtcList));
    sendTelemetry("DTC", "ACTIVE", "DTC", dtcList, "", "WARN");
  }
}

void queryAndSendVin() {
  const uint8_t payload[] = {read_VehicleInfo, read_VIN};
  IsoTp::Payload response{};

  if (!OBD2Handler::requestPayload(read_VehicleInfo, payload, sizeof(payload), read_VIN, response)) {
    if (OBD2Handler::lastResponseWasNegative()) {
      last_vin_text = Obd::Diagnostics::lastNegativeResponse();
      WebConsoleHandler::log("[VIN] Negative response: " + last_vin_text);
      sendTelemetry("STATUS", "VIN", "VIN", last_vin_text.c_str(), "", "ERROR");
    } else {
      last_vin_text = "TIMEOUT";
      WebConsoleHandler::log("[VIN] Timeout waiting for ISO-TP response");
      sendTelemetry("STATUS", "VIN", "VIN", "N/A", "", "TIMEOUT");
    }
    Obd::Diagnostics::setVin(last_vin_text.c_str());
    return;
  }

  char vin[24] = {};
  if (Obd::decodeVin(response.bytes.data(), response.length, vin, sizeof(vin))) {
    last_can_msg_timestamp = millis();
    last_obd_response_time = millis();
    can_bus_active = true;
    last_vin_text = vin;
    Obd::Diagnostics::setVin(vin);
    WebConsoleHandler::log("[VIN] " + String(vin));
    sendTelemetry("STATUS", "VIN", "VIN", vin, "", "OK");
  } else {
    last_vin_text = "INVALID";
    Obd::Diagnostics::setVin("INVALID");
    WebConsoleHandler::log("[VIN] Invalid Mode 09 PID 02 response");
    sendTelemetry("STATUS", "VIN", "VIN", "N/A", "", "ERROR");
  }
}

void queryAndSendUdsDiagnostics() {
  if (!Config::Sender::EnableUDS) return;

  udsClient.setRequestId(IsoTp::PhysicalRequestId);

  Uds::Response response{};
  if (udsClient.testerPresent(response)) {
    sendStatusFrame("UDS", "TESTER_PRESENT", "OK");
  } else {
    sendStatusFrame("UDS", response.negative ? Uds::Diagnostics::lastNegativeResponse() : "NO_RESPONSE",
                    response.negative ? "ERROR" : "WARN");
  }

  if (udsClient.readDataByIdentifier(kUdsVinDid, response)) {
    char vin[24] = {};
    if (Uds::decodeAsciiDid(response.data, response.length, kUdsVinDid, vin, sizeof(vin))) {
      last_uds_did_text = String("VIN=") + vin;
      Uds::Diagnostics::setLastDid(kUdsVinDid, vin);
      WebConsoleHandler::log("[UDS] DID 0xF190 VIN=" + String(vin));
      sendTelemetry("STATUS", "UDS_DID_F190", "UDS_VIN", vin, "", "OK");
    } else {
      last_uds_did_text = "DID_F190_INVALID";
      Uds::Diagnostics::setLastDid(kUdsVinDid, "INVALID");
      WebConsoleHandler::log("[UDS] DID 0xF190 invalid response");
      sendTelemetry("STATUS", "UDS_DID_F190", "UDS_VIN", "N/A", "", "ERROR");
    }
  } else {
    last_uds_did_text = response.negative ? Uds::Diagnostics::lastNegativeResponse() : "TIMEOUT";
    Uds::Diagnostics::setLastDid(kUdsVinDid, last_uds_did_text.c_str());
    sendTelemetry("STATUS", "UDS_DID_F190", "UDS_VIN", last_uds_did_text.c_str(), "", response.negative ? "ERROR" : "TIMEOUT");
  }

  if (udsClient.readDtcInformation(0x02, 0xFF, response)) {
    char summary[48];
    snprintf(summary, sizeof(summary), "len=%u", static_cast<unsigned>(response.length));
    last_uds_dtc_text = summary;
    Uds::Diagnostics::setLastDtcSummary(summary);
    WebConsoleHandler::log("[UDS] DTC information " + String(summary));
    sendTelemetry("DTC", "UDS", "UDS_DTC", summary, "", "OK");
  } else {
    last_uds_dtc_text = response.negative ? Uds::Diagnostics::lastNegativeResponse() : "TIMEOUT";
    Uds::Diagnostics::setLastDtcSummary(last_uds_dtc_text.c_str());
    sendTelemetry("DTC", "UDS", "UDS_DTC", last_uds_dtc_text.c_str(), "", response.negative ? "ERROR" : "TIMEOUT");
  }
}
// End region ================= DTCs lesen =================

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
    if (Config::Debug::TraceSenderTelemetry && Config::Debug::Serial) {
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
  if (!(Config::Debug::TraceSenderTelemetry && Config::Debug::Serial)) return;

  const uint32_t now = millis();
  if (telemetry_last_summary_ms == 0) {
    telemetry_last_summary_ms = now;
    return;
  }

  if (now - telemetry_last_summary_ms < Config::Debug::TraceSummaryIntervalMs) return;

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
  if (!esp_now_ready) return;
  const uint32_t now = millis();
  if (!StatusLogic::isHeartbeatDue(now, last_heartbeat_send_time, kHeartbeatIntervalMs)) return;
  last_heartbeat_send_time = now;
  ++heartbeat_count;

  char uptimeText[16];
  char heartbeatText[16];
  snprintf(uptimeText, sizeof(uptimeText), "%lu", static_cast<unsigned long>(now));
  snprintf(heartbeatText, sizeof(heartbeatText), "%lu", static_cast<unsigned long>(heartbeat_count));

  const bool senderRunning = WebConsoleHandler::isStarted();
  const bool obdRecent = last_obd_response_time > 0 &&
                         now - last_obd_response_time <= Config::Display::ObdTimeoutMs;
  const bool canRecent = last_can_msg_timestamp > 0 &&
                         now - last_can_msg_timestamp <= Config::Display::CanTimeoutMs;

  sendStatusFrame("HEARTBEAT", heartbeatText, "OK");
  sendStatusFrame("SENDER", senderRunning ? "RUNNING" : "WAIT_WEB_START", senderRunning ? "OK" : "WARN");
  sendStatusFrame("FW", Config::Project::FirmwareVersion, "OK");
  sendStatusFrame("UPTIME", uptimeText, "OK");
  sendStatusFrame("CAN", can_driver_ready ? (canRecent ? "ACTIVE" : "IDLE") : "INIT_FAIL",
                  can_driver_ready ? (canRecent ? "OK" : "WARN") : "ERROR");
  sendStatusFrame("OBD", Config::Sender::EnableOBD2 ? (obdRecent ? "ACTIVE" : "NO_RESPONSE") : "DISABLED",
                  Config::Sender::EnableOBD2 ? (obdRecent ? "OK" : "WARN") : "WARN");
  sendStatusFrame("UDS", Config::Sender::EnableUDS ? (Uds::Diagnostics::available() ? "AVAILABLE" : "UNKNOWN") : "DISABLED",
                  Config::Sender::EnableUDS ? (Uds::Diagnostics::available() ? "OK" : "WARN") : "WARN");
  sendStatusFrame("SIM", Simulation::RuntimeSimulation::enabled() ? "ACTIVE" : "INACTIVE",
                  Simulation::RuntimeSimulation::enabled() ? "OK" : "WARN");
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
  WebConsoleRuntimeStatus status;
  status.canActive = can_bus_active;
  status.obdActive = Config::Sender::EnableOBD2 && last_obd_response_time > 0 &&
                     millis() - last_obd_response_time <= Config::Display::ObdTimeoutMs;
  status.pidSupportReady = supported_pids_initialized;
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
  status.obdState = Config::Sender::EnableOBD2 ? (status.obdActive ? "ACTIVE" : "NO_RESPONSE") : "DISABLED";
  status.espNowState = esp_now_ready ? "READY" : "INIT_FAIL";
  status.lastSendError = last_send_error_text;
  status.lastDtc = last_dtc_text;
  status.lastVin = last_vin_text;
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
  status.lastUdsDid = Uds::Diagnostics::lastDid();
  status.lastUdsDtc = Uds::Diagnostics::lastDtcSummary();
  WebConsoleHandler::updateRuntimeStatus(status);
}

// end region ================ PRINT_CAN_FLAG ================

// region ================ LED-Test ================
void handleLedTestButton() {
  const bool pressed = digitalRead(kButtonPin) == LOW;
  if (pressed == led_test_active) return;
  if (millis() - last_led_test_change_time < Config::Sender::LedTestDebounceMs) return;

  led_test_active = pressed;
  last_led_test_change_time = millis();
  digitalWrite(kLedPin1, pressed ? HIGH : LOW);
  digitalWrite(kLedPin2, pressed ? HIGH : LOW);
  WebConsoleHandler::log(pressed ? "[Sender] LED-Test aktiv" : "[Sender] LED-Test beendet");
}

// end region ================ LED-Test ================

// region ================================== loop ==================================
void SenderApp::tick() {
  currentMillis = millis();
  OTAHandler::handleOTA();
  WebConsoleHandler::handle();
  updateWebConsoleStatus();
  handleLedTestButton();
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

  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(kPollingRateMs));

  //------------- CAN Empfang -------------
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    CANHandler::processIncoming();
    last_can_msg_timestamp = millis();
  }
  //---------------------------------------

  //------------ Fehleranzeigen ------------
  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  bool ledShouldBeOn = false;  // Variable to determine if LED should blink

  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      debugln("Alert: TWAI controller has become error passive.");
      last_error_text = "CAN error passive";
      ledShouldBeOn = true;
      debugln(" CAN MSG..........ERROR");
      sendStatusFrame("CAN", "ERROR_PASSIVE", "WARN");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      debugln("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      last_error_text = "CAN bus error";
      ledShouldBeOn = true;
      debugln(" CAN MSG......BUS ERROR");
      sendStatusFrame("CAN", "BUS_ERROR", "ERROR");
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      debugln("Alert: The RX queue is full causing a received frame to be lost.");
      last_error_text = "CAN RX queue full";
      ledShouldBeOn = true;
      debugln(" CAN MSG.........Q FULL");
      sendStatusFrame("CAN", "RX_QUEUE_FULL", "ERROR");
  }
  //---------------------------------------

  //------------- LED Steuerung ------------
      // blink LED if needed
  if (ledShouldBeOn) {
    digitalWrite(kLedPin1, HIGH);
    led_last_on_timestamp = currentMillis;
  }

  if (currentMillis - led_last_on_timestamp >= 1000) {
    digitalWrite(kLedPin1, LOW);
    led_last_on_timestamp = 0;
  }
  //---------------------------------------

  //------------- OBD2 Daten regelmäßig abfragen -------------
  if (Config::Sender::EnableOBD2 && currentMillis - last_obd_request_time >= Config::Sender::ObdIntervalMs) {
    can_bus_active = (currentMillis - last_can_msg_timestamp) <= kCanIdleTimeoutMs;
    sendStatusFrame("CAN", can_bus_active ? "ACTIVE" : "IDLE", can_bus_active ? "OK" : "TIMEOUT");

    if (!supported_pids_initialized ||
        currentMillis - last_supported_pid_refresh_time >= kSupportedPidRefreshIntervalMs) {
      refreshSupportedPIDs();
      last_supported_pid_refresh_time = currentMillis;
    }

    static bool unsupported_pid_reported[Config::ObdPidCount] = {false};
    if (supported_pids_initialized) {
      for (size_t i = 0; i < Config::ObdPidCount; i++) {
        if (!isPIDSupported(Config::ObdRequestedPids[i])) {
          if (!unsupported_pid_reported[i]) {
            char pidHex[4];
            snprintf(pidHex, sizeof(pidHex), "%02X", Config::ObdRequestedPids[i]);
            sendTelemetry("OBD", pidHex, getPIDName(Config::ObdRequestedPids[i]), "N/A", "", "UNSUPPORTED");
            unsupported_pid_reported[i] = true;
          }
          continue;
        }
        if (OBD2Handler::requestAndSendPID(Config::ObdRequestedPids[i])) {
          last_obd_response_time = currentMillis;
          last_can_msg_timestamp = currentMillis;
          can_bus_active = true;
        }
      }
    } else {
      sendStatusFrame("OBD", "PID_SUPPORT_TIMEOUT", "WARN");
    }

    if (currentMillis - last_dtc_query_time >= kDtcQueryIntervalMs) {
      queryAndSendDTCs();
      last_dtc_query_time = currentMillis;
    }

    if (currentMillis - last_vin_query_time >= kVinQueryIntervalMs) {
      queryAndSendVin();
      last_vin_query_time = currentMillis;
    }

    if (Config::Sender::EnableUDS &&
        currentMillis - last_uds_query_time >= kUdsQueryIntervalMs) {
      queryAndSendUdsDiagnostics();
      last_uds_query_time = currentMillis;
    }

    last_obd_request_time = currentMillis;
  }
  //----------------------------------------------------------

  //--------------TWAI Status-----------
  // Optional zur Fehlersuche
  if (Config::Debug::Twai &&
      currentMillis - last_twai_status_log_time >= SenderConfig::TwaiStatusLogIntervalMs) {
    CANHandler::printStatus(); // Aktivieren / Deaktivieren: #define TWAI_DEBUG_FLAG 0
    last_twai_status_log_time = currentMillis;
  }
  //-------------------------------------------------------

  //-------------- Fahrzeugstatus & Sleep-Logik -------------
  SenderPower::updateCarStatus();  // Spannung lesen & Status setzen
  SenderPower::handleSleep();      // Verzögerung + Timeout abwarten, ggf. DeepSleep
  //-------------------------------------------------------

  //------------- Batteriespannung senden -----------
  if (millis() - last_battery_send_time > kBatterySendIntervalMs) {
  SenderPower::sendBatteryVoltage();
  last_battery_send_time = millis();
  }
  //-------------------------------------------------

  //------------Reduziert Leistung des ESP-------------
  //SenderPower::reduceHeat(); // Optional
  //-------------------------------------------------------
}
// End region ================================== loop ==================================
