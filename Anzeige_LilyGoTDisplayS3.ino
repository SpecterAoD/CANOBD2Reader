#include <Arduino.h>
#include <TFT_eSPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include "driver/ledc.h"

// TFT Setup
TFT_eSPI tft = TFT_eSPI();

// Backlight Setup
uint8_t brightnessLevels[] = {60, 180, 255};
int brightnessIndex = 1;
const int buttonPin = 0;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

// GPIO für Display Backlight
#define BACKLIGHT_PIN 38

// Aktueller RPM-Wert
volatile uint16_t currentRPM = 0;

// Struktur für empfangene Daten (z. B. uint16_t RPM)
typedef struct struct_message {
  uint16_t rpm;
} struct_message;

void setBacklight(uint8_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void setupBacklight() {
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 10000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = {
    .channel = LEDC_CHANNEL_0,
    .duty = brightnessLevels[brightnessIndex],
    .gpio_num = BACKLIGHT_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .hpoint = 0,
    .timer_sel = LEDC_TIMER_0,
    .intr_type = LEDC_INTR_DISABLE
  };
  ledc_channel_config(&ledc_channel);

  setBacklight(brightnessLevels[brightnessIndex]);
}

// Callback für empfangene Daten
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  struct_message receivedData;
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  currentRPM = receivedData.rpm;
}

void setup() {
  Serial.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  setupBacklight();

  // ESP-NOW starten
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler beim Initialisieren von ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onDataRecv); // Callback setzen
}

void loop() {
  // Display aktualisieren
  tft.fillScreen(TFT_BLACK);

  tft.setTextDatum(MC_DATUM); // Mitte-Mitte Bezugspunkt
  tft.setTextSize(3);         // Größere Schriftgröße
  tft.drawString(String(currentRPM) + " RPM", tft.width() / 2, tft.height() / 2);

  delay(200); // Kurzes Delay für flüssige Anzeige
}