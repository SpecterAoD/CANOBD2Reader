#include "SenderPower.h"
#include <Arduino.h>
#include "config/SenderConfig.h"
#include "Utils.h"

namespace {
  constexpr int CarIsRunning = 1;
  constexpr int CarIsOff = 0;

  int carStatus = CarIsOff;
  uint32_t lastCarRunningTime = 0;
  uint32_t sleepTriggerTime = 0;
  float lastVoltage = 0.0f;

  struct VoltageMeasurement {
    int adcRaw = 0;
    float voltage = 0.0f;
  };

  VoltageMeasurement readVinVoltage() {
    VoltageMeasurement measurement;
    measurement.adcRaw = analogRead(SenderConfig::VoltageDividerPin);
    const float adcVoltage = measurement.adcRaw * (3.3f / 4095.0f);
    measurement.voltage = adcVoltage * SenderConfig::VoltageCalcFactor;
    lastVoltage = measurement.voltage;
    return measurement;
  }

  void enterDeepSleep() {
    esp_sleep_enable_timer_wakeup(static_cast<uint64_t>(SenderConfig::SleepPeriodSec) * 1000000ULL);
    esp_deep_sleep_start();
  }
}

namespace SenderPower {
  void reduceHeat() {
    setCpuFrequencyMhz(SenderConfig::CpuFrequency);
  }

  void updateCarStatus() {
    VoltageMeasurement measurement = readVinVoltage();
    const int detectedStatus = measurement.voltage > 13.2f ? CarIsRunning : CarIsOff;

    if (detectedStatus == CarIsRunning) {
      carStatus = CarIsRunning;
      lastCarRunningTime = millis();
      sleepTriggerTime = 0;
      return;
    }

    if (carStatus == CarIsRunning &&
        millis() - lastCarRunningTime < SenderConfig::StartStopDelayMs &&
        measurement.voltage > (12.0f + SenderConfig::VoltageChangeThreshold)) {
      return;
    }

    carStatus = CarIsOff;
  }

  void handleSleep() {
    if (carStatus == CarIsRunning) {
      sleepTriggerTime = 0;
      return;
    }

    if (sleepTriggerTime == 0) {
      sleepTriggerTime = millis();
      return;
    }

    if (millis() - sleepTriggerTime >= SenderConfig::StartStopDelayMs) {
      enterDeepSleep();
    }
  }

  void sendBatteryVoltage() {
    VoltageMeasurement measurement = readVinVoltage();
    char value[16];
    snprintf(value, sizeof(value), "%.2f", static_cast<double>(measurement.voltage));
    Utils::sendTelemetry("BATTERY", "VOLTAGE", "BatteryVoltage", value, "V", "OK");
  }

  float getLastVoltage() {
    if (lastVoltage <= 0.01f) {
      readVinVoltage();
    }
    return lastVoltage;
  }

  bool isCarRunning() {
    return carStatus == CarIsRunning;
  }
}
