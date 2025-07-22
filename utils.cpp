// utils.cpp - implementations for utility functions and globals
#include "utils.h"
#include <Arduino.h>

// global status variables defined here
int car_status = CAR_IS_OFF;
uint32_t currentMillis;
unsigned long last_can_msg_timestamp = 0;
unsigned long last_engine_off_time = 0;

// voltage measurement through voltage divider
// returns raw ADC average (see table below for reference values)
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
double get_vin_voltage() {
  int avg = 0;
  for (int i = 0; i < 10; i++) {
    avg += analogRead(SHIELD_VOLTAGE_DIVIDER);
  }
  avg /= 10;
  return avg;
}

// determine vehicle status based on battery voltage
void update_car_status() {
  int avg_voltage = get_vin_voltage();

  if (avg_voltage > 3500) {
    if (car_status != CAR_IS_RUNNING) debugln("Fahrzeugstatus: RUNNING");
    car_status = CAR_IS_RUNNING;
    digitalWrite(SHIELD_LED_PIN, HIGH);
  } else if (avg_voltage > 3250) {
    // in-between voltage range - ignore
  } else {
    if (car_status != CAR_IS_OFF) {
      debugln("Fahrzeugstatus: OFF");
      last_engine_off_time = currentMillis;
    }
    car_status = CAR_IS_OFF;
    digitalWrite(SHIELD_LED_PIN, LOW);
  }
}

// put ESP32 into deep sleep
void esp_deep_sleep() {
  esp_sleep_enable_timer_wakeup(SLEEP_PERIOD * 1000000);
  debugln(" ESP32........... SLEEP");
  debugln("------------------------");
  esp_deep_sleep_start();
}

// run sleep logic including start/stop delay
void handleSleep() {
  if (car_status == CAR_IS_OFF &&
      (currentMillis - last_engine_off_time > START_STOP_DELAY_MS) &&
      (currentMillis - last_can_msg_timestamp > CAN_IDLE_TIMEOUT)) {
    esp_deep_sleep();
  }
}

// optional heat reduction settings
void reduceHeat() {
  btStop();
  setCpuFrequencyMhz(CPU_FREQUENCY);
  WiFi.setTxPower(WIFI_POWER_11dBm);
}
