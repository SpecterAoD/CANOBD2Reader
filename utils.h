// utils.h – LED, Spannung, Sleep & Fahrzeugstatus
#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "config.h"

// === Globale Statusvariablen ===
extern int car_status;
extern uint32_t currentMillis;
extern unsigned long last_can_msg_timestamp;
extern unsigned long last_engine_off_time;

// === Spannung messen (über Spannungsteiler am ADC) ===
double get_vin_voltage();

// === Fahrzeugstatus anhand Batteriespannung bestimmen ===
void update_car_status();

// === Deep Sleep aktivieren ===
void esp_deep_sleep();

// === Sleep-Logik ausführen (inkl. Start/Stopp-Verzögerung) ===
void handleSleep();

// === Temperatur, Bluetooth etc. optimieren (optional) ===
void reduceHeat();

#endif
