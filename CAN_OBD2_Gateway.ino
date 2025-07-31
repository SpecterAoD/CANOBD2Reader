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
#include <esp_now.h>
#include <WiFi.h>
#include "driver/twai.h"
#include "PIDs.h"
#include "PID_Converter.h"
// End region ================================== Includes ==================================

// region ================================== #define ==================================
// --------- Firmeware ---------
#define FIRMWARE_VER "3.1203"
// -----------------------------

// ------ Debug Optionen -------
#define BUDRATE 115200
// uncomment #define DEBUG_FLAG  1 to debug or troubleshoot issues
// leave it commented for normal use

//#define PRINT_CAN_FLAG  1
#define DEBUG_FLAG 0       		// 0 = aus, 1 = an
#define TWAI_DEBUG_FLAG 0       // 0 = aus, 1 = an
#define POWER_DEBUG_FLAG 0      // 0 = aus, 1 = an

#if DEBUG_FLAG
  #define debug(...) Serial.print(__VA_ARGS__)
  #define debugln(...) Serial.println(__VA_ARGS__)
#else
  #define debug(...)
  #define debugln(...)
#endif

#if POWER_DEBUG_FLAG
  #define power_debug(...) Serial.print(__VA_ARGS__)
  #define power_debugln(...) Serial.println(__VA_ARGS__)
#else
  #define power_debug(...)
  #define power_debugln(...)
#endif
// -----------------------------

// ------ TWAI Modus auswählen ------
// TWAI_MODE_NORMAL     → Senden + Empfangen (z. B. für OBD2)
// TWAI_MODE_LISTEN_ONLY → Nur Empfangen (z. B. CAN-Sniffer)
#define ENABLE_CAN      1  // CAN empfangen/senden
#define ENABLE_OBD2     1  // 1 = aktiv, 0 = deaktiviert

#if ENABLE_OBD2
  #define TWAI_OPERATION_MODE TWAI_MODE_NORMAL
#else
  #define TWAI_OPERATION_MODE TWAI_MODE_LISTEN_ONLY
#endif
// -----------------------------

// ---- MAX_PAYLOAD_LEN --------
#define MAX_PAYLOAD_LEN 128
// -----------------------------

// ------------ Pins ------------
#define SHIELD_LED_PIN 26
#define SHIELD_LED_PIN2 27
#define SHIELD_BUTTON_PIN 25
#define SHIELD_CAN_RX 4
#define SHIELD_CAN_TX 5
#define SHIELD_VOLTAGE_DIVIDER 32

// ------------ CAN / OBD2 ------------
#define POLLING_RATE_MS 500
#define CAN_IDLE_TIMEOUT 500
#define SLEEP_PERIOD 6 // 6
#define RAW_DATA_ONLY 0  // 1 = nur Rohdaten senden, 0 = berechnete Werte senden

// ----------- Fahrzeug Status -----------
#define CAR_IS_RUNNING 1
#define CAR_IS_OFF 0
// -----------------------------

// ----------- Power -----------
#define START_STOP_DELAY_MS 300000 // 5 Minuten Wartezeit
#define CPU_FREQUENCY 80 // CPU Frequenz im Sleep-Modus
#define VOLTAGE_CALCULATION_FAKTOR 4.79   // 4.7 Spannungsteiler // 4.84 aus Tabelle errechnet
#define VOLTAGE_CHANGE_THRESHOLD 0.2 // z. B. 0.2 V Unterschied

// --- ESP-NOW Sicherheit ---
#define USE_ESPNOW_ENCRYPTION true

// 16 Byte AES-Schlüssel (unbedingt geheim halten!)
#define ESPNOW_AES_KEY { \
  0x3A, 0x7F, 0xC2, 0x91, 0x18, 0x5D, 0xE0, 0xB3, \
  0x4C, 0x22, 0xA1, 0x6E, 0xD4, 0x0F, 0x97, 0x8B \
}

// Ziel-MAC festlegen (kein Broadcast!)
#define ESPNOW_PEER_MAC { 0xF0, 0xF5, 0xBD, 0x43, 0x29, 0x20 }
// ----------------------------- 
// End region ================================== #define ==================================

// region ================================== struct ==================================
typedef struct {
  char payload[MAX_PAYLOAD_LEN];  // Textbasierter Frame für OBD2-Daten
  uint16_t crc;
} esp_now_text_frame_t;

typedef struct {
  uint8_t magic = 0x42;
  uint32_t timestamp;
  int mesh_id;                    // Eindeutige ID des Geräts
  unsigned long can_id;          // CAN-Identifier
  byte len;                       // Länge der CAN-Daten
  uint8_t d[8];                   // CAN-Daten (max. 8 Byte)
  uint16_t crc;
} esp_now_frame_t;

// ------------ Struct zur Spannungsmessung ------------
typedef struct {
  int adc_raw;
  float voltage;
} VoltageMeasurement;
// End region ================================== struct ==================================

// region ========================== Laufzeitvariablen ==========================
uint32_t led_last_on_timestamp = 0;
uint32_t currentMillis;
unsigned long last_can_msg_timestamp = 0;
unsigned long last_obd_request_time = 0;
unsigned long last_car_running_time = 0;
int car_status = 0;
unsigned long sleep_trigger_time  = 0;
unsigned long last_battery_send_time = 0;
float last_fuel_rate = 0.0;    // L/h
float last_speed = 0.0;        // km/h
float consumption_sum = 0.0;   // Summe für Mittelwert
int consumption_count = 0;     // Anzahl Messungen

// End region ========================== Laufzeitvariablen ==========================

// region ========================== OBD2-Konfiguration ==========================
// Moegliche Abfragen siehe PIDs.h
const byte obd_requested_pids[] = {
  //ENGINE_RPM
  ENGINE_COOLANT_TEMP,
  VEHICLE_SPEED,
  CONTROL_MODULE_VOLTAGE,
  ENGINE_FUEL_RATE,
  ENGINE_OIL_TEMP
};
const byte obd_pid_count = sizeof(obd_requested_pids) / sizeof(obd_requested_pids[0]);
const unsigned long OBD_INTERVAL_MS = 2000;

// End region ========================== OBD2-Konfiguration ==========================

// region ================ CRC ================
uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int j = 0; j < 8; ++j) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}
// end region ================ CRC ================

// region ================================== ESP-NOW ==================================
int esp_now_mesh_id;
uint8_t peerAddress[6] = ESPNOW_PEER_MAC;

esp_now_text_frame_t text_frame;
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
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = USE_ESPNOW_ENCRYPTION;

  #if USE_ESPNOW_ENCRYPTION
  uint8_t lmk[16] = ESPNOW_AES_KEY;
  memcpy(peerInfo.lmk, lmk, 16);
  #endif

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
void setup() {
#ifdef DEBUG_FLAG
  Serial.begin(BUDRATE);
  debugln(BUDRATE);
#endif

  pinMode(SHIELD_LED_PIN, OUTPUT);
  pinMode(SHIELD_LED_PIN2, OUTPUT);
  pinMode(SHIELD_BUTTON_PIN, INPUT_PULLUP);  // Taster zieht gegen GND
  pinMode(SHIELD_VOLTAGE_DIVIDER, INPUT);

  debugln("\n------------------------");
  debugln("         SpecterAoD");
  debugln("       CAN OBD2 SHIELD");
  debugln("      based on MrDiy.ca");
  debugln("https://gitlab.com/MrDIYca/canabus");
  debugln("------------------------");

  last_can_msg_timestamp = millis() - CAN_IDLE_TIMEOUT + 5;

  if (!initESPNow() || (!initCAN())) {
    debugln(" SYSTEM......... FAIL");
    debugln(" SYSTEM......... FAIL");
    debugln(" ... restarting");
    delay(60 * 1000);
    ESP.restart();
  }

  digitalWrite(SHIELD_LED_PIN, HIGH);
}
// End region ================================== Setup ==================================

// region ================================== initCAN ==================================
bool initCAN() {
  debugln(" CAN...............INIT");
  debug(" TWAI Modus: ");
if (TWAI_OPERATION_MODE == TWAI_MODE_NORMAL) {
  debugln("NORMAL (Senden & Empfangen)");
} else {
  debugln("LISTEN_ONLY (Nur Empfangen)");
}

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)SHIELD_CAN_TX, (gpio_num_t)SHIELD_CAN_RX, TWAI_OPERATION_MODE);
  g_config.rx_queue_len = 32;
  g_config.tx_queue_len = 2;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    debugln(" CAN....Install......OK");

  } else {
    debugln(" CAN....Driver....FAIL");
    return false;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    debugln(" CAN....Start........OK");

  } else {
    debugln(" CAN....Start......FAIL");
    return false;
  }

  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    debugln(" CAN....Alerts.......OK");
  } else {
    debugln(" CAN....Alerts.....FAIL");
    return false;
  }
  debugln(" CAN.................OK");
  return true;
}
// End region ================================== initCAN ==================================

// region ================================== CAN Nachricht verarbeiten ==================================
static void handleCANMessage(twai_message_t& message) {
  esp_now_frame_t can_frame;
  can_frame.timestamp = millis();
  can_frame.mesh_id = esp_now_mesh_id;
  can_frame.can_id = message.identifier;
  can_frame.len = message.data_length_code;
  memcpy(can_frame.d, message.data, message.data_length_code);

  // CRC16 berechnen
  can_frame.crc = crc16((uint8_t*)&can_frame, sizeof(esp_now_frame_t) - 2);

  esp_now_send(peerAddress, (uint8_t*)&can_frame, sizeof(can_frame));

  debug("← CAN ID: 0x");
  debug(message.identifier, HEX);
  debug(" Data: ");
  for (int i = 0; i < message.data_length_code; i++) {
    if (message.data[i] < 0x10) debug("0");
    debug(message.data[i], HEX);
    debug(" ");
  }
  debugln("");
}
// End region ================================== CAN Nachricht verarbeiten ==================================

// region ================= OBD2 Anfrage senden =================
bool sendOBDRequest(byte mode, byte pid) {
  twai_message_t request = {};
  request.identifier = 0x7DF;
  request.extd = 0;
  request.rtr = 0;
  request.data_length_code = 8;
  request.data[0] = 0x02; // Länge des Dateninhalts
  request.data[1] = mode; // Mode (0x01 = Live Data)
  request.data[2] = pid;  // Angefragter PID
  for (int i = 3; i < 8; i++) request.data[i] = 0x55; // Padding

  /*
  debug("→ OBD REQ: Mode=0x");
  debug(mode, HEX);
  debug(" PID=0x");
  debugln(pid, HEX);

  return twai_transmit(&request, pdMS_TO_TICKS(10)) == ESP_OK;
  */
  debug("→ OBD SEND: ");
  debug("ID=0x"); debug(request.identifier, HEX);
  debug(" Data=");
  for (int i = 0; i < 8; i++) {
    debug(" ");
    debug(request.data[i], HEX);
  }

  esp_err_t result = twai_transmit(&request, pdMS_TO_TICKS(10));

  if (result == ESP_OK) {
    debugln(" [OK]");
    return true;
  } else {
    debug(" [FAIL: ");
    switch (result) {
      case ESP_ERR_TIMEOUT: debug("TIMEOUT"); break;
      case ESP_ERR_INVALID_STATE: debug("INVALID_STATE"); break;
      case ESP_FAIL: debug("GENERAL FAIL"); break;
      default: debug("CODE "); debug(result);
    }
    debugln("]");
    return false;
  }
}
// End region ================= OBD2 Anfrage senden =================

// region ================= OBD2 Antwort empfangen =================
bool receiveOBDResponse(byte mode, byte pid, byte* outData, byte& outLen) {
  unsigned long start = millis();
  while (millis() - start < 200) {
    twai_message_t response;
    if (twai_receive(&response, pdMS_TO_TICKS(10)) == ESP_OK) {
      
      
      // Immer alle empfangenen Nachrichten ausgeben
      debug("← RAW RECV ID=0x"); debug(response.identifier, HEX);
      debug(" Len="); debug(response.data_length_code);
      debug(" Data=");
      for (int i = 0; i < response.data_length_code; i++) {
        debug(" "); debug(response.data[i], HEX);
      }
      debugln("");


      if (response.identifier >= 0x7E8 && response.identifier <= 0x7EF) {
        if (response.data[1] == (mode | 0x40) && response.data[2] == pid) {
          outLen = response.data_length_code - 3;
          memcpy(outData, &response.data[3], outLen);

          debug(" OBD RESP: ID=0x");
          debug(response.identifier, HEX);
          debug(" Data: ");
          for (byte i = 0; i < outLen; i++) {
            if (outData[i] < 0x10) debug("0");
            debug(outData[i], HEX);
            debug(" ");
          }
          debugln("");

          return true;
        }
      }
    }
  }
  return false;
}
// End region ================= OBD2 Antwort empfangen =================

// reginon ================ calcConsumption ================
void calcConsumption(byte pid, float value) {
    if (pid == VEHICLE_SPEED) last_speed = value;
    if (pid == ENGINE_FUEL_RATE) last_fuel_rate = value;

    // Schutz vor Division durch 0
    if (last_speed < 1.0) return;
    if (last_fuel_rate < 0.1) return;

    // Verbrauch in L/100 km
    float consumption = (last_fuel_rate / last_speed) * 100.0;
    consumption_sum += consumption;
    consumption_count++;

    // --- Wenn 10 Werte gesammelt → Mittelwert senden ---
    if (consumption_count >= 10) {
        float avg_consumption = consumption_sum / consumption_count;

        snprintf(text_frame.payload, sizeof(text_frame.payload),
                 "FUEL_CONS,AVG,%.2f,L100", avg_consumption);
        text_frame.crc = crc16((uint8_t*)&text_frame, sizeof(text_frame) - 2);
        esp_now_send(peerAddress, (uint8_t*)&text_frame, sizeof(text_frame));

        consumption_sum = 0;
        consumption_count = 0;
    }
}
// end reginon ================ calcConsumption ================

// region ================= OBD2 Anfrage senden und Antwort verarbeiten =================
void sendOBDData(byte pid, const char* name, const char* value) {
  snprintf(text_frame.payload, sizeof(text_frame.payload),
           "%02X,%s,%s", pid, name, value);
  text_frame.crc = crc16((uint8_t*)&text_frame, sizeof(text_frame) - 2);
  esp_now_send(peerAddress, (uint8_t*)&text_frame, sizeof(text_frame));
}
// --- Die Hauptfunktion ---
void requestAndSendOBDPID(byte pid) {
  // ------------ Anfrage senden ------------
  if (!sendOBDRequest(read_LiveData, pid)) {
    debug("→ OBD PID 0x");
    debug(pid, HEX);
    debug(" [");
    debug(getPIDName(pid));
    debugln("] SEND FAIL");

    sendOBDData(pid, "ERROR", "N/A");
    return;
  }
  // ------------ Antwort empfangen ------------
  byte responseData[8];
  byte responseLen = 0;

  if (receiveOBDResponse(read_LiveData, pid, responseData, responseLen)) {

#if RAW_DATA_ONLY
    // Rohdaten senden als HEX-String
    char rawHex[3 * 8 + 1] = {0};
    for (byte i = 0; i < responseLen; i++) {
      sprintf(&rawHex[i * 3], "%02X ", responseData[i]);
    }
    sendOBDData(pid, getPIDName(pid), rawHex);

#else
    // Umrechnen in physikalische Werte
    PIDResult result = convertPID(pid, responseData, responseLen);

    debug("← OBD PID: 0x");
    debug(pid, HEX);
    debug(" [");
    debug(getPIDName(pid));
    debug("] = ");
    debug(result.value);
    debug(" ");
    debugln(result.unit);

    // Verbrauch berechnen
    if (pid == VEHICLE_SPEED || pid == ENGINE_FUEL_RATE) {
      calcConsumption(pid, result.value);
    }

    // Nur PIDs senden, die nicht für Verbrauchsberechnung reserviert sind
    if (pid != VEHICLE_SPEED && pid != ENGINE_FUEL_RATE) {
      char valueStr[16];
      snprintf(valueStr, sizeof(valueStr), "%.2f %s", result.value, result.unit);
      sendOBDData(pid, getPIDName(pid), valueStr);
    }
#endif

  } else {
    debug("← OBD PID: 0x");
    debug(pid, HEX);
    debug(" [");
    debug(getPIDName(pid));
    debugln("] TIMEOUT");

    sendOBDData(pid, "ERROR", "N/A");
  }
}
  // -------------------------------------------
// End region ================= OBD2 Anfrage senden und Antwort verarbeiten =================

// region ======================== Power ===============================
// ------------ ESP32-Stromverbrauch senken (optional) ------------
void reduceHeat() {

  btStop();  // make sure BT is disabled
  setCpuFrequencyMhz(CPU_FREQUENCY);
  WiFi.setTxPower(WIFI_POWER_11dBm);
}
//----------------------------------------------------------------

// ------------ Deep Sleep aktivieren ------------
void handleSleep() {
  if (car_status == CAR_IS_OFF &&
      (currentMillis - last_car_running_time > START_STOP_DELAY_MS) &&
      (currentMillis - last_obd_request_time >= OBD_INTERVAL_MS)) {
      //(currentMillis - last_can_msg_timestamp > CAN_IDLE_TIMEOUT)) {
    esp_deep_sleep();
  }
}

void esp_deep_sleep() {
  esp_sleep_enable_timer_wakeup(SLEEP_PERIOD * 1000000);
  debugln(" ESP32........... SLEEP");
  debugln("------------------------");
  esp_deep_sleep_start();
}
//------------------------------------------------

// ------------ Spannung messen (über Spannungsteiler am ADC) ------------
/* ------------ Messwerttabelle ------------ 
    MEASSURED ADC VALUES for a given voltage
      Volt    ADC
      10.5    2640
      11.31   2850
      11.58   2930
      11.75   2970
      11.9    3000
      12.06   3060
      12.2    3100
      12.32   3150
      12.42   3170
      12.5    3190
      12.6    3220 ---- max 3250 for car to be off

      |||||||||||||||||  gray area   |||||||||||||||

      13.5    3520 ---- min 3500 for car to be running
      14.7    4030
	  ------------------------------------------------
  */ 
VoltageMeasurement get_vin_voltage() {
  static const int NUM_SAMPLES = 10;
  static int samples[NUM_SAMPLES] = {0};
  static int index = 0;
  static long sum = 0;
  static bool initialized = false;

  int raw = analogRead(SHIELD_VOLTAGE_DIVIDER);

  if (!initialized) {
    for (int i = 0; i < NUM_SAMPLES; i++) samples[i] = raw;
    sum = raw * NUM_SAMPLES;
    initialized = true;
  }

  sum -= samples[index];
  samples[index] = raw;
  sum += raw;
  index = (index + 1) % NUM_SAMPLES;

  int avg = sum / NUM_SAMPLES;

  VoltageMeasurement result;
  result.adc_raw = avg;
  result.voltage = (avg / 4095.0) * 3.3 * VOLTAGE_CALCULATION_FAKTOR;
  return result;
}
// ------------ Fahrzeugstatus anhand Batteriespannung bestimmen ------------
void update_car_status() {
  // Nur für 12V Blei-Säure-Batterien geeignet
  VoltageMeasurement v = get_vin_voltage();
  int avg_voltage = v.adc_raw;

  if (avg_voltage > 3400) { //3500
    car_status = CAR_IS_RUNNING;
    last_car_running_time = millis(); // Zeit merken
    digitalWrite(SHIELD_LED_PIN, HIGH);
  } else if (avg_voltage > 3000) { //3250
    // Graubereich – keine Änderung
  } else {
    car_status = CAR_IS_OFF;
    digitalWrite(SHIELD_LED_PIN, LOW);
    // Optional: Einschlafen, wenn Spannung sehr niedrig ist
  }
}

//------------------------------------------------------------------------
void sendBatteryVoltage() {
  VoltageMeasurement v = get_vin_voltage();
  float voltage = v.voltage;

  static float last_sent_voltage = 0;
  float delta = fabs(voltage - last_sent_voltage);

  // Nur bei signifikanter Änderung senden (z. B. >0.1 V)
  if (delta < VOLTAGE_CHANGE_THRESHOLD && last_sent_voltage != 0) return;

  last_sent_voltage = voltage;

  // Logging / Anzeige
  power_debug("ADC-Wert: "); power_debugln(v.adc_raw);
  power_debug("Batteriespannung: "); power_debug(voltage); power_debugln(" V");

  // Optional: Warnung bei Grenzwerten
  if (voltage < 11.5 || voltage > 14.8) {
    snprintf(text_frame.payload, sizeof(text_frame.payload),
             "BATTERY,ALERT,%.2f,V", voltage);
  } else {
    snprintf(text_frame.payload, sizeof(text_frame.payload),
             "BATTERY,VOLTAGE,%.2f,V", voltage);
  }

  text_frame.crc = crc16((uint8_t*)&text_frame, sizeof(text_frame) - 2);
  esp_now_send(peerAddress, (uint8_t*)&text_frame, sizeof(text_frame));
}

// End region ======================== Power ===============================

// region ================ PRINT_CAN_FLAG ================
#ifdef PRINT_CAN_FLAG
void printFrame(twai_message_t& message) {

  if (message.extd) {
    Serial.print("CAX: 0x");
  } else {
    Serial.print("CAN: 0x");
  }
  Serial.print(message.identifier, HEX);
  Serial.print(" (");
  Serial.print(message.identifier, DEC);
  Serial.print(")");
  Serial.print(" [");
  Serial.print(message.data_length_code, DEC);
  Serial.print("] <");
  for (int i = 0; i < message.data_length_code; i++) {
    if (i != 0) Serial.print(":");
    Serial.print(message.data[i], HEX);
  }
  Serial.println(">");
}

void printCAN(esp_now_frame_t* message) {

  Serial.print("ESP ");
  Serial.print(message->mesh_id);
  Serial.print(": ");
  Serial.print(" 0x");
  Serial.print(message->can_id, HEX);
  Serial.print(" (");
  Serial.print(message->can_id, DEC);
  Serial.print(")");
  Serial.print(" [");
  Serial.print(message->len, DEC);
  Serial.print("] <");
  for (int i = 0; i < message->len; i++) {
    if (i != 0) Serial.print(":");
    Serial.print(message->d[i], HEX);
  }
  Serial.print(">");
}
#endif
// End region ================ PRINT_CAN_FLAG ================

// region ================ TWAI Status ================
#if TWAI_DEBUG_FLAG
void printTWAIStatus() {
  twai_status_info_t status;
  if (twai_get_status_info(&status) == ESP_OK) {
    debugln("------- TWAI Status -------");

    debug(" Controller State: ");
    switch (status.state) {
      case TWAI_STATE_STOPPED:     debugln("STOPPED"); break;
      case TWAI_STATE_RUNNING:     debugln("RUNNING"); break;
      case TWAI_STATE_BUS_OFF:     debugln("BUS OFF"); break;
      case TWAI_STATE_RECOVERING:  debugln("RECOVERING"); break;
      default:                     debugln("UNKNOWN"); break;
    }

    debug(" TX Failed: ");           debugln(status.tx_failed_count);
    debug(" RX Missed (Overflow): ");debugln(status.rx_missed_count);
    debug(" Bus Errors: ");          debugln(status.bus_error_count);
    debug(" Arbitration Lost: ");    debugln(status.arb_lost_count);

    debugln("---------------------------");
  } else {
    debugln("Fehler beim Abfragen des TWAI-Status.");
  }
}
#endif
// End region ================ TWAI Status ================

// reginon ================ LED-Test ================
void ledtest() {
  digitalWrite(SHIELD_LED_PIN, LOW);
  digitalWrite(SHIELD_LED_PIN2, LOW);
  blinkLED(SHIELD_LED_PIN);
  delay(500);
  blinkLED(SHIELD_LED_PIN2);
}
void blinkLED(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delay(50);
  digitalWrite(pin, LOW);
}
// end reginon ================ LED-Test ================

// region ================================== loop ==================================
void loop() {
  currentMillis = millis();
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));

  //------------- CAN Empfang -------------
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handleCANMessage(message);
    }
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
      ledShouldBeOn = true;
      debugln(" CAN MSG..........ERROR");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      debugln("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      //Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
      ledShouldBeOn = true;
      debugln(" CAN MSG......BUS ERROR");
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      debugln("Alert: The RX queue is full causing a received frame to be lost.");
      //Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
      //Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
      //Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
      ledShouldBeOn = true;
      debugln(" CAN MSG.........Q FULL");
  }
  //---------------------------------------

  //------------- LED Steuerung ------------
      // blink LED if needed
  if (ledShouldBeOn) {
    digitalWrite(SHIELD_LED_PIN, HIGH);
    led_last_on_timestamp = currentMillis;
  }

  if (currentMillis - led_last_on_timestamp >= 1000) {
    digitalWrite(SHIELD_LED_PIN, LOW);
    led_last_on_timestamp = 0;
  }
  //---------------------------------------

  //------------- OBD2 Daten regelmäßig abfragen -------------
  if (ENABLE_OBD2 && currentMillis - last_obd_request_time >= OBD_INTERVAL_MS) {
    for (byte i = 0; i < obd_pid_count; i++) {
      requestAndSendOBDPID(obd_requested_pids[i]);
    }
    last_obd_request_time = currentMillis;
  }
  //----------------------------------------------------------

  //--------------TWAI Status-----------
  // Optional zur Fehlersuche
  #if TWAI_DEBUG_FLAG
    printTWAIStatus(); // Aktievieren / Deaktivieren  #define TWAI_DEBUG_FLAG 0
  #endif
  //-------------------------------------------------------

  //-------------- Fahrzeugstatus & Sleep-Logik -------------
  update_car_status();  // Spannung lesen & Status setzen (LED + car_status)
  handleSleep();        // Verzögerung + Timeout abwarten → ggf. DeepSleep
  //-------------------------------------------------------

  //------------- Batteriespannung senden -----------
  if (millis() - last_battery_send_time > 3000) {  //5000
  sendBatteryVoltage();
  last_battery_send_time = millis();
  }
  //-------------------------------------------------

  // ------------LED-Test-------------
  // Optional
  ///*
  if (digitalRead(SHIELD_BUTTON_PIN) == LOW) {
    debugln("Taster wurde gedrückt! LED-Test");
    ledtest();
  }
  //*/
  //-------------------------------------------------------
  //------------Reduziert Leistung des ESP-------------
  //reduceHeat(); //Optional
  //-------------------------------------------------------
}
// End region ================================== loop ==================================
