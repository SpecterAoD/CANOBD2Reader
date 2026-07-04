#pragma once

#include "ActivityMonitor.h"
#include "SenderLoopState.h"

namespace SenderPower {
  void updateActivity(const Runtime::SenderLoopState& state, uint32_t lastObdResponseAt);
  void publishPowerTelemetry();
  void sendBatteryVoltage();
  void reduceHeat();
  float getLastVoltage();
  bool isCarRunning();
  const Power::ActivitySnapshot& activitySnapshot();
  const char* vehicleStateName();
  const char* powerCommandName();
}
