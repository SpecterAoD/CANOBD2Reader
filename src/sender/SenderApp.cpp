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
constexpr uint32_t kObdResponseTimeoutMs = Config::Sender::ObdResponseTimeoutMs;
constexpr uint32_t kObdTxTimeoutMs = Config::Sender::ObdTxTimeoutMs;
constexpr uint32_t kBatterySendIntervalMs = Config::Sender::BatterySendIntervalMs;
constexpr uint32_t kSupportedPidRefreshIntervalMs = Config::Sender::SupportedPidRefreshIntervalMs;
constexpr uint32_t kDtcQueryIntervalMs = Config::Sender::DtcQueryIntervalMs;
constexpr uint32_t kSimulationSendIntervalMs = Config::Sender::SimulationIntervalMs;
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
unsigned long last_supported_pid_refresh_time = 0;
unsigned long last_dtc_query_time = 0;
unsigned long last_simulation_send_time = 0;
uint32_t telemetry_sequence = 0;
uint32_t telemetry_send_ok = 0;
uint32_t telemetry_send_fail = 0;
uint32_t telemetry_last_summary_ms = 0;
size_t simulation_sample_index = 0;
bool can_bus_active = false;
bool can_driver_ready = false;
bool supported_pids_initialized = false;
uint32_t supported_pids_01_20 = 0;
uint32_t supported_pids_21_40 = 0;
uint32_t supported_pids_41_60 = 0;
String last_dtc_text = "--";
String last_error_text = "";

// End region ========================== Laufzeitvariablen ==========================

// region ========================== Funktionsprototypen ==========================
int getMeshID();
void sendTelemetry(const char* type, const char* key, const char* name, const char* value, const char* unit, const char* status);
void sendStatusFrame(const char* key, const char* value, const char* status);
void updateWebConsoleStatus();
void maybeLogTelemetrySendSummary();
void refreshSupportedPIDs();
bool isPIDSupported(byte pid);
void queryAndSendDTCs();
void sendSimulationTelemetry();
void blinkLED(uint8_t pin);
// End region ========================== Funktionsprototypen ==========================

// region ================================== ESP-NOW ==================================
int esp_now_mesh_id;
uint8_t peerAddress[6] = {};

EspNowTelemetryFrame text_frame;
char last_telemetry_payload[kMaxPayloadLength] = {};
esp_now_peer_info_t peerInfo = {};

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
    memcpy(peerInfo.lmk, Config::Network::EspNowAesKey, sizeof(Config::Network::EspNowAesKey));
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
  Serial.begin(Config::Project::Baudrate);
  debugln(Config::Project::Baudrate);

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
  can_driver_ready = Config::Feature::EnableSenderTelemetrySimulation || CANHandler::init();

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

  if (espNowReady) {
    sendTelemetry("STATUS", "FW", "Firmware", Config::Project::FirmwareVersion, "", "OK");
    char protocolVersion[4];
    snprintf(protocolVersion, sizeof(protocolVersion), "%u", Config::Project::ProtocolVersion);
    sendTelemetry("STATUS", "PROTO", "Protocol", protocolVersion, "", "OK");
    sendStatusFrame("CAN", can_driver_ready ? "READY" : "INIT_FAIL", can_driver_ready ? "OK" : "ERROR");
  }

  WebConsoleHandler::log(can_driver_ready ? "Sender bereit" : "Sender im Fehler-/OTA-Modus");
  digitalWrite(kLedPin1, HIGH);
}
// End region ================================== Setup ==================================

// region ================= DTC Anfrage senden =================
bool sendDTCRequest() {
  twai_message_t request = {};
  request.identifier = 0x7DF;
  request.extd = 0;
  request.rtr = 0;
  request.data_length_code = 8;
  request.data[0] = 0x01;      // Ein Datenbyte: Mode 03
  request.data[1] = read_DTCs;
  for (int i = 2; i < 8; i++) request.data[i] = 0x55;

  esp_err_t result = twai_transmit(&request, pdMS_TO_TICKS(kObdTxTimeoutMs));
  return result == ESP_OK;
}
// End region ================= DTC Anfrage senden =================

bool receiveDTCResponse(byte* outData, byte& outLen) {
  unsigned long start = millis();
  while (millis() - start < kObdResponseTimeoutMs) {
    twai_message_t response;
    if (twai_receive(&response, pdMS_TO_TICKS(10)) == ESP_OK) {
      if (response.identifier >= 0x7E8 && response.identifier <= 0x7EF &&
          response.data_length_code >= 3 && response.data_length_code <= 8 &&
          response.data[1] == (read_DTCs | 0x40)) {
        last_can_msg_timestamp = millis();
        can_bus_active = true;

        outLen = response.data_length_code - 2;
        if (outLen > 6) outLen = 6;
        memcpy(outData, &response.data[2], outLen);
        return true;
      }
    }
  }
  return false;
}

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
  sendStatusFrame("PID_SUPPORT", ok ? "READY" : "UNKNOWN", ok ? "OK" : "WARN");
}
// End region ================= Supported PIDs =================

// region ================= DTCs lesen =================
char dtcTypeChar(byte highBits) {
  switch (highBits & 0x03) {
    case 0: return 'P';
    case 1: return 'C';
    case 2: return 'B';
    default: return 'U';
  }
}

void appendDTC(char* buffer, size_t bufferSize, byte a, byte b) {
  if (a == 0 && b == 0) return;

  char code[8];
  snprintf(code, sizeof(code), "%c%01X%03X",
           dtcTypeChar(a >> 6),
           (a >> 4) & 0x03,
           ((a & 0x0F) << 8) | b);

  const size_t used = strlen(buffer);
  if (used >= bufferSize - 1) return;

  snprintf(buffer + used, bufferSize - used, "%s%s", used > 0 ? " " : "", code);
}

void queryAndSendDTCs() {
  byte responseData[8];
  byte responseLen = 0;

  if (!sendDTCRequest()) {
    last_dtc_text = "SEND_FAIL";
    sendTelemetry("DTC", "ACTIVE", "DTC", "N/A", "", "SEND_FAIL");
    return;
  }

  if (!receiveDTCResponse(responseData, responseLen)) {
    last_dtc_text = "TIMEOUT";
    sendTelemetry("DTC", "ACTIVE", "DTC", "N/A", "", "TIMEOUT");
    return;
  }

  char dtcList[64] = {0};
  for (byte i = 0; i + 1 < responseLen; i += 2) {
    appendDTC(dtcList, sizeof(dtcList), responseData[i], responseData[i + 1]);
  }

  if (dtcList[0] == '\0') {
    last_dtc_text = "Keine";
    sendTelemetry("DTC", "ACTIVE", "DTC", "Keine", "", "OK");
  } else {
    last_dtc_text = dtcList;
    sendTelemetry("DTC", "ACTIVE", "DTC", dtcList, "", "WARN");
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
  const esp_err_t sendResult = esp_now_send(peerAddress, (uint8_t*)&text_frame, sizeof(text_frame));
  if (sendResult == ESP_OK) {
    ++telemetry_send_ok;
  } else {
    ++telemetry_send_fail;
    last_error_text = "ESP-NOW send error " + String(static_cast<int>(sendResult));
    if (Config::Debug::TraceSenderTelemetry && Config::Debug::Serial) {
      Serial.printf("[sender-tx] send failed seq=%lu err=%d\n",
                    static_cast<unsigned long>(telemetry_sequence),
                    static_cast<int>(sendResult));
    }
  }

  maybeLogTelemetrySendSummary();
  WebConsoleHandler::recordTelemetry(last_telemetry_payload);
  if (strcmp(status, "OK") != 0) {
    last_error_text = String(type) + "/" + key + ": " + status;
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

  Serial.printf("[sender-tx] seq=%lu ok=%lu fail=%lu sim=%s\n",
                static_cast<unsigned long>(telemetry_sequence),
                static_cast<unsigned long>(telemetry_send_ok),
                static_cast<unsigned long>(telemetry_send_fail),
                Config::Feature::EnableSenderTelemetrySimulation ? "on" : "off");
  telemetry_last_summary_ms = now;
}

void sendStatusFrame(const char* key, const char* value, const char* status) {
  sendTelemetry("STATUS", key, key, value, "", status);
}

void sendSimulationTelemetry() {
  if (millis() - last_simulation_send_time < kSimulationSendIntervalMs) return;
  last_simulation_send_time = millis();

  const SimulationData::Sample& sample =
      SimulationData::Samples[simulation_sample_index % SimulationData::SampleCount];

  char value[16];
  const float simulatedValue = SimulationData::valueForSample(sample, millis(), simulation_sample_index);
  snprintf(value, sizeof(value), sample.decimals == 0 ? "%.0f" : "%.1f", simulatedValue);
  sendTelemetry(sample.type, sample.key, sample.name, value, sample.unit, "OK");

  simulation_sample_index++;

  // Status-, CAN- und DTC-Pakete werden zwischen die Messwerte gemischt, damit
  // alle Display-Seiten ohne Fahrzeug und ohne CAN-Transceiver getestet werden koennen.
  if (simulation_sample_index % SimulationData::SampleCount == 0) {
    sendStatusFrame("CAN", "SIMULATED", "OK");
    sendTelemetry("CAN", "RAW", "LastCAN", "0x7E8 DLC8 04 41 0C 1A F8 55 55 55", "", "OK");
    sendTelemetry("CAN", "HINT", "CANHint", "Simulierte OBD Antwort RPM", "", "OK");
    sendTelemetry("CAN", "COUNT", "CANCount", "128", "frames", "OK");
    sendTelemetry("DTC", "ACTIVE", "DTC", "P0133 P0420", "", "WARN");
  }
}
// end region ================ Telemetrie senden ================

void updateWebConsoleStatus() {
  WebConsoleRuntimeStatus status;
  status.canActive = can_bus_active;
  status.obdActive = Config::Sender::EnableOBD2;
  status.pidSupportReady = supported_pids_initialized;
  status.simulationActive = Config::Feature::EnableSenderTelemetrySimulation;
  status.batteryVoltage = SenderPower::getLastVoltage();
  status.uptimeMs = millis();
  status.telemetrySequence = telemetry_sequence;
  status.lastCanAgeMs = last_can_msg_timestamp == 0 ? 0 : millis() - last_can_msg_timestamp;
  status.canState = can_driver_ready ? (can_bus_active ? "ACTIVE" : "IDLE") : "INIT_FAIL";
  status.lastDtc = last_dtc_text;
  status.lastTelemetry = String(last_telemetry_payload);
  status.lastError = last_error_text;
  WebConsoleHandler::updateRuntimeStatus(status);
}

// end region ================ PRINT_CAN_FLAG ================

// region ================ LED-Test ================
void ledtest() {
  digitalWrite(kLedPin1, LOW);
  digitalWrite(kLedPin2, LOW);
  blinkLED(kLedPin1);
  delay(500);
  blinkLED(kLedPin2);
}
void blinkLED(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delay(50);
  digitalWrite(pin, LOW);
}
// end region ================ LED-Test ================

// region ================================== loop ==================================
void SenderApp::tick() {
  currentMillis = millis();
  OTAHandler::handleOTA();
  WebConsoleHandler::handle();
  updateWebConsoleStatus();

  if (Config::Feature::EnableSenderTelemetrySimulation) {
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
    return;
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
      //Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
      ledShouldBeOn = true;
      debugln(" CAN MSG......BUS ERROR");
      sendStatusFrame("CAN", "BUS_ERROR", "ERROR");
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      debugln("Alert: The RX queue is full causing a received frame to be lost.");
      last_error_text = "CAN RX queue full";
      //Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
      //Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
      //Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
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
      OBD2Handler::requestAndSendPID(Config::ObdRequestedPids[i]);
    }

    if (currentMillis - last_dtc_query_time >= kDtcQueryIntervalMs) {
      queryAndSendDTCs();
      last_dtc_query_time = currentMillis;
    }

    last_obd_request_time = currentMillis;
  }
  //----------------------------------------------------------

  //--------------TWAI Status-----------
  // Optional zur Fehlersuche
  if (Config::Debug::Twai) {
    CANHandler::printStatus(); // Aktivieren / Deaktivieren: #define TWAI_DEBUG_FLAG 0
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

  // ------------LED-Test-------------
  // Optional
  ///*
  if (digitalRead(kButtonPin) == LOW) {
    debugln("Taster wurde gedrückt! LED-Test");
    ledtest();
  }
  //*/
  //-------------------------------------------------------
  //------------Reduziert Leistung des ESP-------------
  //SenderPower::reduceHeat(); // Optional
  //-------------------------------------------------------
}
// End region ================================== loop ==================================
