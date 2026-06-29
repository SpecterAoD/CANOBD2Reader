#pragma once

namespace SenderPower {
  void updateCarStatus();
  void handleSleep();
  void sendBatteryVoltage();
  void reduceHeat();
  float getLastVoltage();
  bool isCarRunning();
}
