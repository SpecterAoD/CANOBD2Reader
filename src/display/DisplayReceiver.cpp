#include "DisplayReceiver.h"

#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "common_config.h"
#include "config/ProjectConfig.h"
#include "config/DisplayConfig.h"
#include "config/NetworkConfig.h"
#include "config/SecurityConfig.h"
#include "config/LoggingConfig.h"
#include "AuthHelpers.h"
#include "DisplayData.h"
#include "TelemetryCodec.h"
#include "StatusLogic.h"
#include "DiagnosticLog.h"

namespace {

struct QueuedTelemetryPacket {
  uint8_t senderMac[6]{};
  uint16_t length = 0;
  uint8_t bytes[sizeof(Telemetry::TelemetryPacket)]{};
};

QueueHandle_t telemetryQueue = nullptr;

uint32_t rx_packets = 0;
uint32_t rx_crc_errors = 0;
uint32_t rx_invalid_length = 0;
uint32_t rx_mac_filtered = 0;
uint32_t rx_queue_drops = 0;
uint32_t rx_parse_errors = 0;
uint32_t rx_last_summary_ms = 0;

void maybeLogRxSummary() {
  if (!(LoggingConfig::TraceDisplayTelemetry && LoggingConfig::SerialEnabled)) return;

  const uint32_t now = millis();
  if (rx_last_summary_ms == 0) {
    rx_last_summary_ms = now;
    return;
  }
  if (now - rx_last_summary_ms < LoggingConfig::TraceSummaryIntervalMs) return;

  DiagnosticLog::appendf("[display-rx] packets=%lu crc=%lu badlen=%lu macdrop=%lu queuedrop=%lu parse=%lu lastSeq=%lu",
                         static_cast<unsigned long>(rx_packets),
                         static_cast<unsigned long>(rx_crc_errors),
                         static_cast<unsigned long>(rx_invalid_length),
                         static_cast<unsigned long>(rx_mac_filtered),
                         static_cast<unsigned long>(rx_queue_drops),
                         static_cast<unsigned long>(rx_parse_errors),
                         static_cast<unsigned long>(DisplayData::runtime().lastSequence));
  rx_last_summary_ms = now;
}

String fieldAt(const String& raw, uint8_t index) {
  int start = 0;
  for (uint8_t i = 0; i < index; i++) {
    start = raw.indexOf(',', start);
    if (start < 0) return "";
    start++;
  }
  int end = raw.indexOf(',', start);
  if (end < 0) end = raw.length();
  return raw.substring(start, end);
}

uint8_t countFields(const String& raw) {
  uint8_t fields = raw.length() > 0 ? 1 : 0;
  for (uint16_t i = 0; i < raw.length(); i++) {
    if (raw[i] == ',') fields++;
  }
  return fields;
}

void parseTelemetryPayload(const char* payload, uint32_t sequence) {
  using namespace DisplayData;

  const String raw(payload == nullptr ? "" : payload);
  const uint8_t fields = countFields(raw);
  if (fields < 3) {
    ++rx_parse_errors;
    runtime().droppedPackets++;
    runtime().lastError = "Ungueltiges Paket";
    maybeLogRxSummary();
    return;
  }

  String type = fieldAt(raw, 0);
  String key = fieldAt(raw, 1);
  String name = fieldAt(raw, 2);
  String value = fields > 3 ? fieldAt(raw, 3) : "";
  String unit = fields > 4 ? fieldAt(raw, 4) : "";
  String status = fields > 5 ? fieldAt(raw, 5) : "OK";
  // The packet header is authoritative. The CSV sequence is kept for human
  // diagnostics, but packet-loss detection must use the CRC-protected header.
  uint32_t payloadSequence = sequence;

  if (type == "BATTERY" && key == "VOLTAGE" && fields == 4) {
    name = "BatteryVoltage";
    value = fieldAt(raw, 2);
    unit = fieldAt(raw, 3);
    status = "OK";
  }

  if (fields == 3) {
    type = "OBD";
    key = fieldAt(raw, 0);
    name = fieldAt(raw, 1);
    value = fieldAt(raw, 2);
    unit = "";
    status = value == "N/A" ? "TIMEOUT" : "OK";
  }

  if (payloadSequence > 0) {
    runtime().droppedPackets += StatusLogic::packetLossFromSequence(runtime().lastSequence, payloadSequence);
  }
  if (payloadSequence > 0) runtime().lastSequence = payloadSequence;

  upsertValue(type, key, name, value, unit, status, payloadSequence);
  if (type == "STATUS") {
    if (key == "HEARTBEAT") {
      runtime().lastHeartbeatAt = millis();
      runtime().lastHeartbeatSequence = payloadSequence;
      DiagnosticLog::appendf("[DISPLAY] Heartbeat received seq=%lu",
                             static_cast<unsigned long>(payloadSequence));
    } else if (key == "CAN") {
      runtime().lastCanStatusAt = millis();
    } else if (key == "OBD") {
      runtime().lastObdStatusAt = millis();
    } else if (key == "POWER_STATE") {
      runtime().vehicleState = value;
      runtime().lastPowerStatusAt = millis();
    } else if (key == "ACTIVITY_SCORE") {
      runtime().activityScore = static_cast<uint8_t>(value.toInt());
      runtime().lastPowerStatusAt = millis();
    } else if (key == "POWER_COMMAND") {
      runtime().powerCommand = value;
      runtime().displaySleepRequested = value == "Sleep";
      if (value == "Wakeup") runtime().displaySleepRequested = false;
      runtime().lastPowerStatusAt = millis();
    }
  }
  runtime().lastRawPayload = raw;
  runtime().lastError = status == "OK" ? "" : status;
  maybeLogRxSummary();
}

void processQueuedPacket(const QueuedTelemetryPacket& queued) {
  using namespace DisplayData;

  Telemetry::TelemetryPacket packet{};
  const auto status = Telemetry::TelemetryCodec::decode(queued.bytes, queued.length, packet);
  if (status != Telemetry::DecodeStatus::Ok) {
    if (status == Telemetry::DecodeStatus::CrcMismatch) {
      ++rx_crc_errors;
      runtime().crcErrors++;
      runtime().lastError = "CRC-Fehler";
    } else {
      ++rx_invalid_length;
      runtime().droppedPackets++;
      runtime().lastError = "Ungueltiges Telemetriepaket";
    }
    maybeLogRxSummary();
    return;
  }

  char payload[ProjectConfig::TelemetryPayloadSize + 1]{};
  memcpy(payload, packet.payload, packet.payloadLength);
  payload[packet.payloadLength] = '\0';

  ++rx_packets;
  runtime().receivedPackets++;
  runtime().lastReceivedAt = millis();
  parseTelemetryPayload(payload, packet.sequence);
}

void onDataRecv(const uint8_t* senderMac, const uint8_t* incomingData, int len) {
  if (senderMac == nullptr || incomingData == nullptr || len <= 0) return;

  if (memcmp(senderMac, NetworkConfig::SenderAllowedMac, 6) != 0) {
    ++rx_mac_filtered;
    return;
  }

  if (telemetryQueue == nullptr || len > static_cast<int>(sizeof(Telemetry::TelemetryPacket))) {
    ++rx_invalid_length;
    return;
  }

  QueuedTelemetryPacket queued{};
  memcpy(queued.senderMac, senderMac, sizeof(queued.senderMac));
  queued.length = static_cast<uint16_t>(len);
  memcpy(queued.bytes, incomingData, len);

  if (xQueueSend(telemetryQueue, &queued, 0) != pdTRUE) {
    ++rx_queue_drops;
  }
}

}

namespace DisplayReceiver {

void begin() {
  telemetryQueue = xQueueCreate(DisplayConfig::TelemetryQueueLength, sizeof(QueuedTelemetryPacket));
  if (telemetryQueue == nullptr) {
    DisplayData::runtime().lastError = "Telemetry Queue fehlgeschlagen";
    return;
  }

  const String securityWarning = WebSecurity::espNowSecurityWarning();
  if (SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets && securityWarning.length() > 0) {
    DisplayData::runtime().lastError = "ESP-NOW blockiert: " + securityWarning;
    DiagnosticLog::appendf("[DISPLAY] %s", DisplayData::runtime().lastError.c_str());
    return;
  }

  // Keep Display Web-OTA SoftAP visible while the ESP-NOW receiver is active.
  // The station side is disconnected only from infrastructure WiFi; SoftAP
  // remains up on NetworkConfig::EspNowChannel.
  WiFi.mode(CANOBD2_ENABLE_DISPLAY_OTA ? WIFI_AP_STA : WIFI_STA);
  WiFi.disconnect(false, false);

  if (esp_now_init() != ESP_OK) {
    DisplayData::runtime().lastError = "ESP-NOW Init fehlgeschlagen";
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, NetworkConfig::SenderAllowedMac, 6);
  peer.channel = NetworkConfig::EspNowChannel;
  peer.encrypt = NetworkConfig::UseEspNowEncryption;
  if (NetworkConfig::UseEspNowEncryption) {
    memcpy(peer.lmk, NetworkConfig::EspNowAesKey, 16);
  }

  esp_err_t peerResult = esp_now_add_peer(&peer);
  if (peerResult != ESP_OK && peerResult != ESP_ERR_ESPNOW_EXIST) {
    DisplayData::runtime().lastError = "ESP-NOW Peer fehlgeschlagen";
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  DisplayData::runtime().lastError = "Warte auf Sender";
}

void processQueuedPackets() {
  if (telemetryQueue == nullptr) return;

  QueuedTelemetryPacket queued{};
  while (xQueueReceive(telemetryQueue, &queued, 0) == pdTRUE) {
    processQueuedPacket(queued);
  }
}

}
