#include "DisplayReceiver.h"
#include "DisplayData.h"
#include "Config.h"
#include <WiFi.h>
#include <esp_now.h>

namespace {
  using TextFrame = TelemetryProtocol::TextFrame;

  uint32_t rx_text_frames = 0;
  uint32_t rx_can_frames = 0;
  uint32_t rx_crc_errors = 0;
  uint32_t rx_invalid_length = 0;
  uint32_t rx_mac_filtered = 0;
  uint32_t rx_parse_errors = 0;
  uint32_t rx_last_summary_ms = 0;

  void maybeLogRxSummary() {
    if (!(Config::Debug::TraceDisplayTelemetry && Config::Debug::Serial)) return;

    const uint32_t now = millis();
    if (rx_last_summary_ms == 0) {
      rx_last_summary_ms = now;
      return;
    }

    if (now - rx_last_summary_ms < Config::Debug::TraceSummaryIntervalMs) return;

    Serial.printf("[display-rx] text=%lu can=%lu crc=%lu badlen=%lu macdrop=%lu parse=%lu lastSeq=%lu\n",
                  static_cast<unsigned long>(rx_text_frames),
                  static_cast<unsigned long>(rx_can_frames),
                  static_cast<unsigned long>(rx_crc_errors),
                  static_cast<unsigned long>(rx_invalid_length),
                  static_cast<unsigned long>(rx_mac_filtered),
                  static_cast<unsigned long>(rx_parse_errors),
                  static_cast<unsigned long>(DisplayData::lastSequence));
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

  void parseTelemetryPayload(const String& raw) {
    using namespace DisplayData;
    const uint8_t fields = countFields(raw);
    if (fields < 3) {
      ++rx_parse_errors;
      droppedPackets++;
      lastError = "Ungültiges Paket";
      maybeLogRxSummary();
      return;
    }

    String type = fieldAt(raw, 0);
    String key = fieldAt(raw, 1);
    String name = fieldAt(raw, 2);
    String value = fields > 3 ? fieldAt(raw, 3) : "";
    String unit = fields > 4 ? fieldAt(raw, 4) : "";
    String status = fields > 5 ? fieldAt(raw, 5) : "OK";
    uint32_t sequence = fields > 6 ? fieldAt(raw, 6).toInt() : 0;

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

    if (sequence > 0 && lastSequence > 0 && sequence > lastSequence + 1) {
      droppedPackets += sequence - lastSequence - 1;
    }
    if (sequence > 0) lastSequence = sequence;

    upsertValue(type, key, name, value, unit, status, sequence);
    lastRawPayload = raw;
    lastError = status == "OK" ? "" : status;
    maybeLogRxSummary();
  }

  void onDataRecv(const uint8_t* senderMac, const uint8_t* incomingData, int len) {
    using namespace DisplayData;
    if (memcmp(senderMac, Config::Network::SenderAllowedMac, 6) != 0) {
      ++rx_mac_filtered;
      droppedPackets++;
      maybeLogRxSummary();
      return;
    }

    if (len == sizeof(DisplayCanFrame)) {
      DisplayCanFrame canFrame;
      memcpy(&canFrame, incomingData, sizeof(canFrame));
      if (TelemetryProtocol::crc16((uint8_t*)&canFrame, sizeof(canFrame) - 2) != canFrame.crc) {
        ++rx_crc_errors;
        crcErrors++;
        lastError = "CAN CRC-Fehler";
        maybeLogRxSummary();
        return;
      }

      char raw[64];
      size_t offset = snprintf(raw, sizeof(raw), "0x%03lX DLC%u",
                               static_cast<unsigned long>(canFrame.canId),
                               static_cast<unsigned int>(canFrame.len));
      for (uint8_t i = 0; i < canFrame.len && i < 8 && offset < sizeof(raw); i++) {
        offset += snprintf(raw + offset, sizeof(raw) - offset, " %02X", canFrame.data[i]);
      }

      receivedPackets++;
      ++rx_can_frames;
      lastReceivedAt = millis();
      upsertValue("CAN", "RAW", "LastCAN", raw, "", "OK", receivedPackets);
      upsertValue("CAN", "COUNT", "CANCount", String(receivedPackets), "frames", "OK", receivedPackets);
      upsertValue("CAN", "HINT", "CANHint", "CAN Rohdaten", "", "OK", receivedPackets);
      maybeLogRxSummary();
      return;
    }

    if (len != sizeof(TextFrame)) {
      ++rx_invalid_length;
      droppedPackets++;
      maybeLogRxSummary();
      return;
    }

    TextFrame frame;
    memcpy(&frame, incomingData, sizeof(frame));
    if (!TelemetryProtocol::validateFrame(frame)) {
      ++rx_crc_errors;
      crcErrors++;
      lastError = "CRC-Fehler";
      maybeLogRxSummary();
      return;
    }

    frame.payload[TelemetryProtocol::MaxPayloadLength - 1] = '\0';
    receivedPackets++;
    ++rx_text_frames;
    lastReceivedAt = millis();
    parseTelemetryPayload(String(frame.payload));
  }
}

namespace DisplayReceiver {
  void begin() {
    WiFi.mode(CANOBD2_ENABLE_DISPLAY_OTA ? WIFI_AP_STA : WIFI_STA);
    WiFi.disconnect(false, false);

    if (esp_now_init() != ESP_OK) {
      DisplayData::lastError = "ESP-NOW Init fehlgeschlagen";
      return;
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, Config::Network::SenderAllowedMac, 6);
    peer.channel = Config::Network::EspNowChannel;
    peer.encrypt = Config::Network::UseEspNowEncryption;
    if (Config::Network::UseEspNowEncryption) {
      memcpy(peer.lmk, Config::Network::EspNowAesKey, sizeof(Config::Network::EspNowAesKey));
    }

    esp_err_t peerResult = esp_now_add_peer(&peer);
    if (peerResult != ESP_OK && peerResult != ESP_ERR_ESPNOW_EXIST) {
      DisplayData::lastError = "ESP-NOW Peer fehlgeschlagen";
      return;
    }

    esp_now_register_recv_cb(onDataRecv);
    DisplayData::lastError = "Warte auf Sender";
  }
}
