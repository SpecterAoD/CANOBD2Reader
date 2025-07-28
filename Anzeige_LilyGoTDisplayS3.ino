#include <Arduino.h>
#include <TFT_eSPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include "driver/ledc.h"
#include <cmath>
#include <SPI.h>

#define DISPLAY_ROTATION 1
#define BACKLIGHT_PIN    38

const float RPM_MIN = 0.0;
const float RPM_MAX = 6000.0;
const float RPM_GREEN_MAX = 3000.0;
const float RPM_ORANGE_MAX = 4500.0;
const float RPM_WARNING_THRESHOLD = 4200.0;

const uint8_t BACKLIGHT_ON = 255;
const int BAR_HEIGHT = 12;
const int BAR_MARGIN_X = 10;
const int BAR_OFFSET_Y = 25;
const float rpmSmoothing = 0.2; // schnelleres Reagieren
const int screenUpdateDelay = 30;

bool darkMode = true;
uint16_t textColorDark = TFT_WHITE;
uint16_t bgColorDark = TFT_BLACK;
uint16_t textColorLight = TFT_BLACK;
uint16_t bgColorLight = TFT_WHITE;

#define MAX_PAYLOAD_LEN 128
#define USE_ESPNOW_ENCRYPTION true

typedef struct {
  char payload[MAX_PAYLOAD_LEN];
  uint16_t crc;
} esp_now_text_frame_t;

const uint8_t allowed_sender_mac[6] = { 0x8C, 0x4B, 0x14, 0x27, 0xEB, 0x48 };
const uint8_t lmk[16] = { 0x3A, 0x7F, 0xC2, 0x91, 0x18, 0x5D, 0xE0, 0xB3, 0x4C, 0x22, 0xA1, 0x6E, 0xD4, 0x0F, 0x97, 0x8B };

uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

TFT_eSPI tft = TFT_eSPI();

String header1 = "", header2 = "", rpmValue = "", rpmUnit = "";
String lastPID = "";
float currentRPM = 0, targetRPM = 0;
float voltageLast = 0.0;
int lastBarWidth = -1;
unsigned long lastReceivedTime = 0;
unsigned long lastVoltageUpdate = 0;
bool simulationActive = false;
unsigned long simStartTime = 0;

void setBacklight(uint8_t brightness) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brightness);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void setupBacklight() {
  ledc_timer_config_t ledc_timer = { LEDC_LOW_SPEED_MODE, LEDC_TIMER_8_BIT, LEDC_TIMER_0, 10000, LEDC_AUTO_CLK };
  ledc_timer_config(&ledc_timer);
  ledc_channel_config_t ledc_channel = { BACKLIGHT_PIN, LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_INTR_DISABLE, LEDC_TIMER_0, 0, 0 };
  ledc_channel_config(&ledc_channel);
  setBacklight(BACKLIGHT_ON);
}

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, allowed_sender_mac, 6);
  peer.channel = 1;
  peer.encrypt = USE_ESPNOW_ENCRYPTION;
  if (USE_ESPNOW_ENCRYPTION) memcpy(peer.lmk, lmk, 16);
  esp_now_add_peer(&peer);
  esp_now_register_recv_cb(onDataRecv);
}

void showBootAnimation() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("R-Line", tft.width() / 2, tft.height() / 2);
  delay(1000);
  tft.fillScreen(TFT_BLACK);
}

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(DISPLAY_ROTATION);
  showBootAnimation();
  tft.fillScreen(darkMode ? bgColorDark : bgColorLight);
  tft.setTextColor(darkMode ? textColorDark : textColorLight);
  setupBacklight();
  setupESPNow();
  simStartTime = millis();
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (memcmp(info->src_addr, allowed_sender_mac, 6) != 0 || len != sizeof(esp_now_text_frame_t)) return;

  esp_now_text_frame_t frame;
  memcpy(&frame, incomingData, sizeof(frame));
  if (crc16((uint8_t*)&frame, sizeof(frame) - 2) != frame.crc) return;

  String raw = String(frame.payload);
  Serial.println("Empfangen: " + raw);

  int c1 = raw.indexOf(','), c2 = raw.indexOf(',', c1 + 1), c3 = raw.indexOf(',', c2 + 1);
  if (c1 <= 0 || c2 <= c1 || c3 <= c2) return;

  header1 = raw.substring(0, c1);
  header2 = raw.substring(c1 + 1, c2);
  rpmValue = raw.substring(c2 + 1, c3);
  rpmUnit  = raw.substring(c3 + 1);

  if (header1 == "BATTERY" && header2 == "VOLTAGE") {
    voltageLast = rpmValue.toFloat();
    lastVoltageUpdate = millis();
  } else {
    if (rpmValue != "ERROR") {
      targetRPM = rpmValue.toFloat();
      simulationActive = false;
    }
    lastPID = header2; // z. B. PID
  }

  lastReceivedTime = millis(); // immer setzen
}

uint16_t getRPMColor(float rpm) {
  float clamped = constrain(rpm, RPM_MIN, RPM_MAX);
  if (clamped <= RPM_GREEN_MAX) return TFT_GREEN;
  if (clamped <= RPM_ORANGE_MAX) return TFT_ORANGE;
  return TFT_RED;
}

void loop() {
  if (simulationActive) {
    float simulated = 3000.0 + 2700.0 * sin((millis() - simStartTime) / 1000.0);
    targetRPM = constrain(simulated, RPM_MIN, RPM_MAX);
    voltageLast = 13.4;
    lastVoltageUpdate = millis();
  }

  currentRPM = std::lerp(currentRPM, targetRPM, rpmSmoothing);
  float clampedRPM = constrain(currentRPM, RPM_MIN, RPM_MAX);
  uint16_t currentBGColor = darkMode ? bgColorDark : bgColorLight;

  // RPM Anzeige
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(3);
  String rpmStr = String(clampedRPM, 0) + " " + rpmUnit;
  int x = tft.width() / 2;
  int y = tft.height() / 2;
  int w = tft.textWidth("0000 rpm", 3);
  int h = 3 * 8 + 4;
  tft.fillRect(x - w / 2, y - h / 2, w, h, currentBGColor);
  tft.setTextColor(TFT_WHITE, currentBGColor);
  tft.drawString(rpmStr, x, y);

  // Fortschrittsbalken
  int barWidth = map(clampedRPM, RPM_MIN, RPM_MAX, 0, tft.width() - 2 * BAR_MARGIN_X);
  if (barWidth != lastBarWidth) {
    lastBarWidth = barWidth;
    int barX = BAR_MARGIN_X;
    int barY = y + BAR_OFFSET_Y;
    tft.fillRect(barX, barY, tft.width() - 2 * BAR_MARGIN_X, BAR_HEIGHT, currentBGColor);
    tft.drawRect(barX - 1, barY - 1, tft.width() - 2 * BAR_MARGIN_X + 2, BAR_HEIGHT + 2, TFT_WHITE);
    tft.fillRect(barX, barY, barWidth, BAR_HEIGHT, getRPMColor(clampedRPM));
  }

  // Spannung oben rechts
  if (millis() - lastVoltageUpdate < 10000 && voltageLast > 0.0) {
    tft.setTextDatum(TR_DATUM);
    tft.setTextSize(2);
    tft.setTextColor(TFT_CYAN, currentBGColor);
    String vstr = String(voltageLast, 2) + " V";
    tft.fillRect(tft.width() - 80, 0, 80, 20, currentBGColor);
    tft.drawString(vstr, tft.width() - 5, 2);
  }

  // Keine Verbindung nur bei echter Funkstille
  if (!simulationActive && millis() - lastReceivedTime > 3000) {
    tft.fillRect(0, 0, tft.width(), 40, currentBGColor);
    tft.setTextDatum(TC_DATUM);
    tft.setTextSize(2);
    tft.setTextColor(TFT_RED, currentBGColor);
    tft.drawString("Keine Verbindung", tft.width() / 2, 10);
    tft.drawString("Warte auf Daten...", tft.width() / 2, 30);
  }

  // PID oder Fehler unten rechts anzeigen
  tft.setTextDatum(BR_DATUM);
  tft.setTextSize(1);
  tft.setTextColor(TFT_DARKGREY, currentBGColor);
  tft.fillRect(tft.width() - 100, tft.height() - 12, 100, 12, currentBGColor);
  tft.drawString("PID: " + lastPID, tft.width() - 2, tft.height() - 2);

  delay(screenUpdateDelay);
}
