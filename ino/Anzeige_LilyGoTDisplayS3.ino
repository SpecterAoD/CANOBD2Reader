#include <Arduino.h>
#include <TFT_eSPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include "driver/ledc.h"
#include <SPI.h>
#if __has_include("TelemetryProtocol.h")
  #include "TelemetryProtocol.h"
#else
  #include "../include/TelemetryProtocol.h"
#endif
#if __has_include("SimulationData.h")
  #include "SimulationData.h"
#else
  #include "../include/SimulationData.h"
#endif

// Anzeige / Empfänger für LilyGo T-Display S3
// Erwartetes Telemetrieformat vom Sender:
// TYPE,KEY,NAME,VALUE,UNIT,STATUS,SEQ

#define DISPLAY_FIRMWARE_VERSION "3.1204"
#define TELEMETRY_PROTOCOL_VERSION 2

#define DISPLAY_ROTATION 1
#define BACKLIGHT_PIN 38
#define NEXT_PAGE_BUTTON_PIN 0

#define MAX_PAYLOAD_LEN 128
#define USE_ESPNOW_ENCRYPTION true
#define ENABLE_DISPLAY_INTERNAL_SIMULATION 0  // 1 = Display ohne Sender mit lokalen Testdaten fuellen

const uint32_t SCREEN_REFRESH_MS = 250;
const uint32_t CONNECTION_TIMEOUT_MS = 3000;
const uint32_t VALUE_TIMEOUT_MS = 5000;
const uint32_t BUTTON_DEBOUNCE_MS = 250;

const uint8_t BACKLIGHT_ON = 255;
const uint8_t PAGE_COUNT = 7;

const uint16_t COLOR_BACKGROUND = TFT_BLACK;
const uint16_t COLOR_PANEL = 0x18E3;       // dunkles Grau/Blau
const uint16_t COLOR_TEXT = TFT_WHITE;
const uint16_t COLOR_MUTED = TFT_DARKGREY;
const uint16_t COLOR_ACCENT = TFT_CYAN;
const uint16_t COLOR_OK = TFT_GREEN;
const uint16_t COLOR_WARN = TFT_ORANGE;
const uint16_t COLOR_ERROR = TFT_RED;

typedef TelemetryProtocol::TextFrame esp_now_text_frame_t;

typedef struct {
  uint8_t magic = 0x42;
  uint32_t timestamp;
  int mesh_id;
  unsigned long can_id;
  byte len;
  uint8_t d[8];
  uint16_t crc;
} esp_now_can_frame_t;

struct TelemetryValue {
  String type;
  String key;
  String name;
  String value;
  String unit;
  String status;
  uint32_t sequence = 0;
  uint32_t updatedAt = 0;
};

const uint8_t allowed_sender_mac[6] = { 0x8C, 0x4B, 0x14, 0x27, 0xEB, 0x48 };
const uint8_t lmk[16] = {
  0x3A, 0x7F, 0xC2, 0x91, 0x18, 0x5D, 0xE0, 0xB3,
  0x4C, 0x22, 0xA1, 0x6E, 0xD4, 0x0F, 0x97, 0x8B
};

TFT_eSPI tft = TFT_eSPI();

TelemetryValue values[28];
uint8_t valueCount = 0;
uint8_t currentPage = 0;
uint8_t lastRenderedPage = 255;
uint32_t lastScreenRefresh = 0;
uint32_t lastReceivedAt = 0;
uint32_t lastButtonAt = 0;
uint32_t receivedPackets = 0;
uint32_t droppedPackets = 0;
uint32_t crcErrors = 0;
uint32_t lastSequence = 0;
String lastRawPayload = "";
String lastError = "Keine Daten";
uint32_t lastInternalSimulationUpdate = 0;
size_t internalSimulationIndex = 0;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);

uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

void setBacklight(uint8_t brightness) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void setupBacklight() {
  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.duty_resolution = LEDC_TIMER_8_BIT;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = 10000;
  timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&timer);

  ledc_channel_config_t channel = {};
  channel.gpio_num = BACKLIGHT_PIN;
  channel.speed_mode = LEDC_LOW_SPEED_MODE;
  channel.channel = LEDC_CHANNEL_0;
  channel.intr_type = LEDC_INTR_DISABLE;
  channel.timer_sel = LEDC_TIMER_0;
  channel.duty = 0;
  ledc_channel_config(&channel);

  setBacklight(BACKLIGHT_ON);
}

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    lastError = "ESP-NOW Init fehlgeschlagen";
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, allowed_sender_mac, 6);
  peer.channel = 1;
  peer.encrypt = USE_ESPNOW_ENCRYPTION;
  if (USE_ESPNOW_ENCRYPTION) memcpy(peer.lmk, lmk, 16);

  esp_err_t peerResult = esp_now_add_peer(&peer);
  if (peerResult != ESP_OK && peerResult != ESP_ERR_ESPNOW_EXIST) {
    lastError = "ESP-NOW Peer fehlgeschlagen";
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  lastError = "Warte auf Sender";
}

TelemetryValue* findValue(const String& name) {
  for (uint8_t i = 0; i < valueCount; i++) {
    if (values[i].name == name || values[i].key == name) return &values[i];
  }
  return nullptr;
}

TelemetryValue* upsertValue(const String& type,
                            const String& key,
                            const String& name,
                            const String& value,
                            const String& unit,
                            const String& status,
                            uint32_t sequence) {
  TelemetryValue* existing = findValue(name);
  if (existing == nullptr) existing = findValue(key);

  if (existing == nullptr) {
    if (valueCount >= (sizeof(values) / sizeof(values[0]))) {
      droppedPackets++;
      lastError = "Werteliste voll";
      return nullptr;
    }
    existing = &values[valueCount++];
  }

  existing->type = type;
  existing->key = key;
  existing->name = name;
  existing->value = value;
  existing->unit = unit;
  existing->status = status;
  existing->sequence = sequence;
  existing->updatedAt = millis();
  return existing;
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
  const uint8_t fields = countFields(raw);
  if (fields < 3) {
    droppedPackets++;
    lastError = "Ungültiges Paket";
    return;
  }

  String type = fieldAt(raw, 0);
  String key = fieldAt(raw, 1);
  String name = fieldAt(raw, 2);
  String value = fields > 3 ? fieldAt(raw, 3) : "";
  String unit = fields > 4 ? fieldAt(raw, 4) : "";
  String status = fields > 5 ? fieldAt(raw, 5) : "OK";
  uint32_t sequence = fields > 6 ? fieldAt(raw, 6).toInt() : 0;

  // Rückwärtskompatibilität für alte Pakete BATTERY,VOLTAGE,12.3,V.
  if (type == "BATTERY" && key == "VOLTAGE" && fields == 4) {
    name = "BatteryVoltage";
    value = fieldAt(raw, 2);
    unit = fieldAt(raw, 3);
    status = "OK";
  }

  // Rückwärtskompatibilität für alte Pakete PID,Name,Wert Einheit.
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
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (memcmp(info->src_addr, allowed_sender_mac, 6) != 0) {
    droppedPackets++;
    return;
  }

  if (len == sizeof(esp_now_can_frame_t)) {
    esp_now_can_frame_t canFrame;
    memcpy(&canFrame, incomingData, sizeof(canFrame));
    if (TelemetryProtocol::crc16((uint8_t*)&canFrame, sizeof(canFrame) - 2) != canFrame.crc) {
      crcErrors++;
      lastError = "CAN CRC-Fehler";
      return;
    }

    char raw[64];
    size_t offset = snprintf(raw, sizeof(raw), "0x%03lX DLC%u",
                             static_cast<unsigned long>(canFrame.can_id),
                             static_cast<unsigned int>(canFrame.len));
    for (uint8_t i = 0; i < canFrame.len && i < 8 && offset < sizeof(raw); i++) {
      offset += snprintf(raw + offset, sizeof(raw) - offset, " %02X", canFrame.d[i]);
    }

    receivedPackets++;
    lastReceivedAt = millis();
    upsertValue("CAN", "RAW", "LastCAN", raw, "", "OK", receivedPackets);
    upsertValue("CAN", "COUNT", "CANCount", String(receivedPackets).c_str(), "frames", "OK", receivedPackets);
    upsertValue("CAN", "HINT", "CANHint", "CAN Rohdaten", "", "OK", receivedPackets);
    return;
  }

  if (len != sizeof(esp_now_text_frame_t)) {
    droppedPackets++;
    return;
  }

  esp_now_text_frame_t frame;
  memcpy(&frame, incomingData, sizeof(frame));
  if (!TelemetryProtocol::validateFrame(frame)) {
    crcErrors++;
    lastError = "CRC-Fehler";
    return;
  }

  frame.payload[MAX_PAYLOAD_LEN - 1] = '\0';
  receivedPackets++;
  lastReceivedAt = millis();
  parseTelemetryPayload(String(frame.payload));
}

bool isConnected() {
  return lastReceivedAt > 0 && millis() - lastReceivedAt <= CONNECTION_TIMEOUT_MS;
}

bool isFresh(const TelemetryValue* value) {
  if (value == nullptr || millis() - value->updatedAt > VALUE_TIMEOUT_MS) return false;
  return value->status == "OK" || value->status == "WARN" || value->status == "ALERT";
}

String displayValue(const char* name, uint8_t decimals = 0) {
  TelemetryValue* value = findValue(String(name));
  if (!isFresh(value)) return "--";

  if (value->value == "N/A" || value->value.length() == 0) return "--";
  float numericValue = value->value.toFloat();
  String result = String(numericValue, decimals);
  if (value->unit.length() > 0) result += " " + value->unit;
  return result;
}

String displayText(const char* name) {
  TelemetryValue* value = findValue(String(name));
  if (value == nullptr || millis() - value->updatedAt > VALUE_TIMEOUT_MS) return "--";
  if (value->value.length() == 0 || value->value == "N/A") return "--";
  return value->value;
}

uint16_t valueColor(const char* name) {
  TelemetryValue* value = findValue(String(name));
  if (!isFresh(value)) return COLOR_MUTED;

  float numericValue = value->value.toFloat();
  String n = String(name);
  if (n == "CoolantTemp" && numericValue >= 105.0f) return COLOR_ERROR;
  if (n == "OilTemp" && numericValue >= 125.0f) return COLOR_ERROR;
  if (n == "BatteryVoltage" && (numericValue < 11.5f || numericValue > 14.8f)) return COLOR_WARN;
  if (n == "RPM" && numericValue >= 4200.0f) return COLOR_WARN;
  return COLOR_TEXT;
}

void drawStatusBar() {
  const uint16_t barColor = isConnected() ? COLOR_PANEL : COLOR_ERROR;
  tft.fillRect(0, 0, tft.width(), 20, barColor);
  tft.setTextSize(1);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(COLOR_TEXT, barColor);
  tft.drawString(isConnected() ? "ESP-NOW OK" : "VERBINDUNG VERLOREN", 4, 10);

  tft.setTextDatum(MR_DATUM);
  String right = "S" + String(currentPage + 1) + "/" + String(PAGE_COUNT);
  if (lastReceivedAt > 0) right += "  " + String((millis() - lastReceivedAt) / 1000) + "s";
  tft.drawString(right, tft.width() - 4, 10);
}

void drawFooter() {
  tft.fillRect(0, tft.height() - 16, tft.width(), 16, COLOR_BACKGROUND);
  tft.setTextSize(1);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(COLOR_MUTED, COLOR_BACKGROUND);
  String footer = "FW ";
  footer += DISPLAY_FIRMWARE_VERSION;
  footer += "  Proto ";
  footer += String(TELEMETRY_PROTOCOL_VERSION);
  tft.drawString(footer, 4, tft.height() - 8);
}

void drawMetricBox(int x, int y, int w, int h, const char* label, const String& value, uint16_t color) {
  tft.fillRoundRect(x, y, w, h, 6, COLOR_PANEL);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED, COLOR_PANEL);
  tft.drawString(label, x + 8, y + 6);

  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(value.length() > 8 ? 2 : 3);
  tft.setTextColor(color, COLOR_PANEL);
  tft.drawString(value, x + w / 2, y + h / 2 + 8);
}

void drawMainPage() {
  drawMetricBox(6, 28, 150, 58, "Geschwindigkeit", displayValue("Speed", 0), valueColor("Speed"));
  drawMetricBox(164, 28, 150, 58, "Drehzahl", displayValue("RPM", 0), valueColor("RPM"));
  drawMetricBox(6, 94, 150, 58, "Kühlmittel", displayValue("CoolantTemp", 0), valueColor("CoolantTemp"));
  drawMetricBox(164, 94, 150, 58, "Bordspannung", displayValue("BatteryVoltage", 1), valueColor("BatteryVoltage"));
}

void drawEnginePage() {
  drawMetricBox(6, 28, 150, 58, "Öltemperatur", displayValue("OilTemp", 0), valueColor("OilTemp"));
  drawMetricBox(164, 28, 150, 58, "Kühlmittel", displayValue("CoolantTemp", 0), valueColor("CoolantTemp"));
  drawMetricBox(6, 94, 150, 58, "Motorlast", displayValue("EngineLoad", 0), valueColor("EngineLoad"));
  drawMetricBox(164, 94, 150, 58, "Ansaugluft", displayValue("IntakeTemp", 0), valueColor("IntakeTemp"));
}

void drawConsumptionPage() {
  drawMetricBox(6, 28, 150, 58, "Ø Verbrauch", displayValue("AverageConsumption", 1), valueColor("AverageConsumption"));
  drawMetricBox(164, 28, 150, 58, "Kraftstoffrate", displayValue("FuelRate", 1), valueColor("FuelRate"));
  drawMetricBox(6, 94, 150, 58, "Geschwindigkeit", displayValue("Speed", 0), valueColor("Speed"));
  drawMetricBox(164, 94, 150, 58, "Drosselklappe", displayValue("Throttle", 0), valueColor("Throttle"));
}

void drawAdditionalPage() {
  drawMetricBox(6, 28, 150, 58, "MAF", displayValue("MAF", 1), valueColor("MAF"));
  drawMetricBox(164, 28, 150, 58, "Tankfüllstand", displayValue("FuelLevel", 0), valueColor("FuelLevel"));
  drawMetricBox(6, 94, 150, 58, "Motorlaufzeit", displayValue("RunTime", 0), valueColor("RunTime"));
  drawMetricBox(164, 94, 150, 58, "Außentemp.", displayValue("AmbientTemp", 0), valueColor("AmbientTemp"));
}

void drawDiagnosticsPage() {
  uint32_t totalPackets = receivedPackets + droppedPackets;
  uint8_t quality = totalPackets == 0 ? 0 : (receivedPackets * 100UL) / totalPackets;
  TelemetryValue* canStatus = findValue("CAN");

  drawMetricBox(6, 28, 150, 42, "ESP-NOW", isConnected() ? "aktiv" : "verloren", isConnected() ? COLOR_OK : COLOR_ERROR);
  bool canStatusRecent = canStatus != nullptr && millis() - canStatus->updatedAt <= VALUE_TIMEOUT_MS;
  drawMetricBox(164, 28, 150, 42, "CAN/OBD", canStatusRecent ? canStatus->value : "--", isFresh(canStatus) ? COLOR_OK : COLOR_WARN);
  drawMetricBox(6, 76, 150, 42, "Datenqualität", String(quality) + " %", quality > 90 ? COLOR_OK : COLOR_WARN);
  drawMetricBox(164, 76, 150, 42, "CRC/Drop", String(crcErrors) + "/" + String(droppedPackets), (crcErrors == 0 && droppedPackets == 0) ? COLOR_OK : COLOR_WARN);

  tft.fillRoundRect(6, 124, 308, 28, 6, COLOR_PANEL);
  tft.setTextDatum(ML_DATUM);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED, COLOR_PANEL);
  tft.drawString("Letztes Paket:", 14, 132);
  tft.setTextColor(COLOR_TEXT, COLOR_PANEL);
  String raw = lastRawPayload.length() > 42 ? lastRawPayload.substring(0, 42) + "..." : lastRawPayload;
  tft.drawString(raw.length() ? raw : lastError, 14, 144);
}

void drawCANPage() {
  drawMetricBox(6, 28, 150, 42, "CAN Frames", displayValue("CANCount", 0), valueColor("CANCount"));
  drawMetricBox(164, 28, 150, 42, "CAN Status", displayText("CAN"), isConnected() ? COLOR_OK : COLOR_WARN);

  tft.fillRoundRect(6, 78, 308, 74, 6, COLOR_PANEL);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED, COLOR_PANEL);
  tft.drawString("Letzter CAN-Frame:", 14, 86);
  tft.setTextColor(COLOR_TEXT, COLOR_PANEL);
  String raw = displayText("LastCAN");
  if (raw.length() > 42) raw = raw.substring(0, 42) + "...";
  tft.drawString(raw, 14, 102);

  tft.setTextColor(COLOR_MUTED, COLOR_PANEL);
  tft.drawString("Auswertung:", 14, 126);
  tft.setTextColor(COLOR_ACCENT, COLOR_PANEL);
  String hint = displayText("CANHint");
  if (hint.length() > 36) hint = hint.substring(0, 36) + "...";
  tft.drawString(hint, 14, 140);
}

void drawDTCPage() {
  TelemetryValue* dtc = findValue("DTC");
  const bool dtcFresh = dtc != nullptr && millis() - dtc->updatedAt <= VALUE_TIMEOUT_MS;
  const bool hasFault = dtcFresh && dtc->status == "WARN" && dtc->value != "Keine";

  drawMetricBox(6, 28, 308, 48, "Fehlercodes / DTC", dtcFresh ? (hasFault ? "AKTIV" : "Keine") : "--",
                hasFault ? COLOR_WARN : (dtcFresh ? COLOR_OK : COLOR_MUTED));

  tft.fillRoundRect(6, 88, 308, 64, 6, COLOR_PANEL);
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED, COLOR_PANEL);
  tft.drawString("Codes:", 14, 96);

  tft.setTextSize(2);
  tft.setTextColor(hasFault ? COLOR_WARN : COLOR_TEXT, COLOR_PANEL);
  String codes = displayText("DTC");
  if (codes.length() > 24) codes = codes.substring(0, 24);
  tft.drawString(codes, 14, 116);

  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED, COLOR_PANEL);
  String age = dtcFresh ? "Letzte DTC-Abfrage: " + String((millis() - dtc->updatedAt) / 1000) + "s" : "Keine aktuelle DTC-Abfrage";
  tft.drawString(age, 14, 142);
}

void updateInternalSimulation() {
  if (!ENABLE_DISPLAY_INTERNAL_SIMULATION) return;
  if (millis() - lastInternalSimulationUpdate < 120) return;
  lastInternalSimulationUpdate = millis();

  const SimulationData::Sample& sample =
      SimulationData::Samples[internalSimulationIndex % SimulationData::SampleCount];

  char value[16];
  const float simulatedValue = SimulationData::valueForSample(sample, millis(), internalSimulationIndex);
  snprintf(value, sizeof(value), sample.decimals == 0 ? "%.0f" : "%.1f", simulatedValue);
  upsertValue(sample.type, sample.key, sample.name, value, sample.unit, "OK", internalSimulationIndex + 1);

  internalSimulationIndex++;
  lastReceivedAt = millis();
  if (internalSimulationIndex % SimulationData::SampleCount == 0) {
    upsertValue("STATUS", "CAN", "CAN", "SIMULATED", "", "OK", internalSimulationIndex);
    upsertValue("CAN", "RAW", "LastCAN", "0x7E8 DLC8 04 41 0C 1A F8 55 55 55", "", "OK", internalSimulationIndex);
    upsertValue("CAN", "HINT", "CANHint", "Lokale CAN-Simulation", "", "OK", internalSimulationIndex);
    upsertValue("CAN", "COUNT", "CANCount", "128", "frames", "OK", internalSimulationIndex);
    upsertValue("DTC", "ACTIVE", "DTC", "P0133 P0420", "", "WARN", internalSimulationIndex);
  }
}

void renderCurrentPage() {
  tft.fillScreen(COLOR_BACKGROUND);
  drawStatusBar();

  switch (currentPage) {
    case 0: drawMainPage(); break;
    case 1: drawEnginePage(); break;
    case 2: drawConsumptionPage(); break;
    case 3: drawAdditionalPage(); break;
    case 4: drawCANPage(); break;
    case 5: drawDiagnosticsPage(); break;
    case 6: drawDTCPage(); break;
  }

  drawFooter();
}

void handleButton() {
  if (digitalRead(NEXT_PAGE_BUTTON_PIN) == LOW && millis() - lastButtonAt > BUTTON_DEBOUNCE_MS) {
    currentPage = (currentPage + 1) % PAGE_COUNT;
    lastRenderedPage = 255;
    lastButtonAt = millis();
  }
}

void showBootScreen() {
  tft.fillScreen(COLOR_BACKGROUND);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(3);
  tft.setTextColor(COLOR_ACCENT, COLOR_BACKGROUND);
  tft.drawString("CAN OBD2", tft.width() / 2, tft.height() / 2 - 12);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_MUTED, COLOR_BACKGROUND);
  tft.drawString("Display " DISPLAY_FIRMWARE_VERSION, tft.width() / 2, tft.height() / 2 + 22);
  delay(900);
}

void setup() {
  Serial.begin(115200);
  pinMode(NEXT_PAGE_BUTTON_PIN, INPUT_PULLUP);

  tft.init();
  tft.setRotation(DISPLAY_ROTATION);
  setupBacklight();
  showBootScreen();
  setupESPNow();
}

void loop() {
  updateInternalSimulation();
  handleButton();

  if (millis() - lastScreenRefresh >= SCREEN_REFRESH_MS || lastRenderedPage != currentPage) {
    renderCurrentPage();
    lastRenderedPage = currentPage;
    lastScreenRefresh = millis();
  }
}
