// ESP-NOW receiver of the display.
//
// The ESP-NOW callback runs in the WiFi task: it only filters by sender MAC
// and copies the raw bytes into a FreeRTOS queue. Decoding and all state
// updates happen in loop() via processQueuedPackets().
#include "DisplayReceiver.h"

#include <WiFi.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "DeviceSecrets.h"
#include "DiagnosticLog.h"
#include "DisplayData.h"
#include "LiveData.h"
#include "StatusLogic.h"
#include "TelemetryCodec.h"
#include "config/NetworkConfig.h"

namespace {

struct QueuedPacket {
  uint16_t length = 0;
  uint8_t bytes[sizeof(Telemetry::TelemetryPacket)]{};
};

QueueHandle_t queue = nullptr;
uint32_t macFiltered = 0;
uint32_t queueDrops = 0;

String fieldAt(const String& raw, uint8_t index) {
  int start = 0;
  for (uint8_t i = 0; i < index; ++i) {
    start = raw.indexOf(',', start);
    if (start < 0) return "";
    ++start;
  }
  int end = raw.indexOf(',', start);
  if (end < 0) end = raw.length();
  return raw.substring(start, end);
}

// Text packet: "TYPE,KEY,NAME,VALUE,UNIT,STATUS[,SEQ]" (TelemetryProtocol).
void handleStatusText(const String& raw, uint32_t sequence) {
  using namespace DisplayData;
  const String type = fieldAt(raw, 0);
  const String key = fieldAt(raw, 1);
  const String name = fieldAt(raw, 2);
  const String value = fieldAt(raw, 3);
  const String unit = fieldAt(raw, 4);
  String status = fieldAt(raw, 5);
  if (type.length() == 0 || key.length() == 0) {
    runtime().droppedPackets++;
    return;
  }
  if (status.length() == 0) status = "OK";

  upsertValue(type, key, name, value, unit, status, sequence);
  runtime().lastRawPayload = raw;
  if (type != "STATUS") return;

  const uint32_t now = millis();
  if (key == "HEARTBEAT") {
    runtime().lastHeartbeatAt = now;
    runtime().lastHeartbeatSequence = sequence;
  } else if (key == "CAN") {
    runtime().lastCanStatusAt = now;
  } else if (key == "OBD") {
    runtime().lastObdStatusAt = now;
  } else if (key == "POWER_STATE") {
    runtime().vehicleState = value;
    runtime().lastPowerStatusAt = now;
  } else if (key == "ACTIVITY_SCORE") {
    runtime().activityScore = static_cast<uint8_t>(value.toInt());
    runtime().lastPowerStatusAt = now;
  } else if (key == "POWER_COMMAND") {
    runtime().powerCommand = value;
    if (value == "Sleep") runtime().displaySleepRequested = true;
    if (value == "Wakeup") runtime().displaySleepRequested = false;
    runtime().lastPowerStatusAt = now;
  }
}

void processPacket(const QueuedPacket& queued) {
  using namespace DisplayData;
  Telemetry::TelemetryPacket packet{};
  const auto result = Telemetry::TelemetryCodec::decode(queued.bytes, queued.length, packet);
  if (result != Telemetry::DecodeStatus::Ok) {
    if (result == Telemetry::DecodeStatus::CrcMismatch) {
      runtime().crcErrors++;
    } else {
      runtime().droppedPackets++;
    }
    if (result == Telemetry::DecodeStatus::InvalidVersion) runtime().lastError = "Sender-Protokoll passt nicht";
    return;
  }

  // The CRC-protected header sequence is authoritative for loss detection.
  runtime().droppedPackets += StatusLogic::packetLossFromSequence(runtime().lastSequence, packet.sequence);
  runtime().lastSequence = packet.sequence;
  runtime().receivedPackets++;
  runtime().lastReceivedAt = millis();
  if (runtime().lastError == "Warte auf Sender") runtime().lastError = "";

  if (packet.type == static_cast<uint8_t>(Telemetry::PacketType::LiveSnapshot)) {
    LiveData::Snapshot snapshot;
    if (LiveData::decodeSnapshot(packet.payload, packet.payloadLength, snapshot)) {
      live().apply(snapshot, millis());
    } else {
      runtime().droppedPackets++;
    }
    return;
  }

  char text[ProjectConfig::TelemetryPayloadSize + 1]{};
  memcpy(text, packet.payload, packet.payloadLength);
  handleStatusText(String(text), packet.sequence);
}

// Arduino-ESP32 3.x (ESP-IDF 5) passes the sender MAC in esp_now_recv_info_t.
void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int length) {
  if (info == nullptr || info->src_addr == nullptr || data == nullptr || length <= 0) return;
  if (memcmp(info->src_addr, Security::secrets().senderMac, 6) != 0) {
    ++macFiltered;
    return;
  }
  if (queue == nullptr || length > static_cast<int>(sizeof(Telemetry::TelemetryPacket))) return;

  QueuedPacket queued;
  queued.length = static_cast<uint16_t>(length);
  memcpy(queued.bytes, data, length);
  if (xQueueSend(queue, &queued, 0) != pdTRUE) ++queueDrops;
}

} // namespace

namespace DisplayReceiver {

void begin() {
  using namespace DisplayData;
  if (!Security::provisioned()) {
    runtime().lastError = "Nicht provisioniert";
    DiagnosticLog::appendf("[DISPLAY] ESP-NOW blockiert: %s", Security::problemText());
    return;
  }

  queue = xQueueCreate(DisplayConfig::TelemetryQueueLength, sizeof(QueuedPacket));
  if (queue == nullptr) {
    runtime().lastError = "Queue fehlgeschlagen";
    return;
  }

  // The management AP (if running) already put the radio in AP+STA mode on
  // the ESP-NOW channel; otherwise station mode is enough for ESP-NOW.
  if (WiFi.getMode() == WIFI_OFF) WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(false, false);

  if (esp_now_init() != ESP_OK) {
    runtime().lastError = "ESP-NOW Init fehlgeschlagen";
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, Security::secrets().senderMac, 6);
  peer.channel = NetworkConfig::EspNowChannel;
  peer.encrypt = NetworkConfig::UseEspNowEncryption;
  if (NetworkConfig::UseEspNowEncryption) memcpy(peer.lmk, Security::secrets().espNowKey, 16);

  const esp_err_t added = esp_now_add_peer(&peer);
  if (added != ESP_OK && added != ESP_ERR_ESPNOW_EXIST) {
    runtime().lastError = "ESP-NOW Peer fehlgeschlagen";
    return;
  }
  esp_now_register_recv_cb(onReceive);
  runtime().lastError = "Warte auf Sender";
}

void processQueuedPackets() {
  if (queue == nullptr) return;
  QueuedPacket queued;
  // Bounded so a flood of packets cannot stall rendering.
  for (uint8_t count = 0; count < DisplayConfig::TelemetryQueueLength && xQueueReceive(queue, &queued, 0) == pdTRUE;
       ++count) {
    processPacket(queued);
  }
}

uint32_t macFilteredCount() { return macFiltered; }
uint32_t queueDropCount() { return queueDrops; }

} // namespace DisplayReceiver
