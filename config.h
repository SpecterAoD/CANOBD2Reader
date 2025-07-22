// config.h – zentrale Definitionen & Einstellungen
#ifndef CONFIG_H
#define CONFIG_H

#include "PIDs.h"

// ==== Firmware Info ====
#define FIRMWARE_VER "0.0.1"

// ==== Debug ====
#define BUDRATE 115200
//#define PRINT_CAN_FLAG  1
#define DEBUG_FLAG 1       // Serielle Ausgabe aktivieren

#ifdef DEBUG_FLAG
  #define debug(...) Serial.print(__VA_ARGS__)
  #define debugln(...) Serial.println(__VA_ARGS__)
#else
  #define debug(...)
  #define debugln(...)
#endif

// ==== Feature-Schalter ====
// TWAI_MODE_NORMAL     → Senden + Empfangen (z. B. für OBD2)
// TWAI_MODE_LISTEN_ONLY → Nur Empfangen (z. B. CAN-Sniffer)
#define ENABLE_CAN 		0  // CAN empfangen/senden
#define ENABLE_OBD2     1  // 1 = aktiv, 0 = deaktiviert
#if ENABLE_OBD2
  #define TWAI_OPERATION_MODE TWAI_MODE_NORMAL
#else
  #define TWAI_OPERATION_MODE TWAI_MODE_LISTEN_ONLY
#endif


// === OBD2 Abfrageintervall (in ms) ===
#define OBD_INTERVAL_MS 2000

// === Liste der OBD2-PIDs, die regelmäßig abgefragt werden sollen ===
const byte OBD_PIDS_TO_QUERY[] = {
  ENGINE_RPM
  //ENGINE_COOLANT_TEMP,
  //VEHICLE_SPEED,
  //CONTROL_MODULE_VOLTAGE,
  //ENGINE_FUEL_RATE,
  //ENGINE_OIL_TEMP
};
const byte NUM_OBD_PIDS = sizeof(OBD_PIDS_TO_QUERY) / sizeof(OBD_PIDS_TO_QUERY[0]);


// ==== Pins ====
#define SHIELD_LED_PIN         26
#define SHIELD_CAN_RX          4
#define SHIELD_CAN_TX          5
#define SHIELD_VOLTAGE_DIVIDER 32

// ==== CAN Einstellungen ====
#define POLLING_RATE_MS     500
#define CAN_IDLE_TIMEOUT    500     // ms ohne CAN-Aktivität vor Sleep

// ==== Sleep ====
#define SLEEP_PERIOD 6               // Sekunden für DeepSleep
#define START_STOP_DELAY_MS 300000  // 5 Minuten Verzögerung vor Sleep (wegen Start-Stopp)
#define CPU_FREQUENCY 80 // CPU Frequenz im Sleep-Modus

// ==== Fahrzeugstatus ====
#define CAR_IS_RUNNING 1
#define CAR_IS_OFF     0
extern int car_status;
extern uint32_t currentMillis;
extern unsigned long last_can_msg_timestamp;

#endif
