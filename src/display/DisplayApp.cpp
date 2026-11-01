// CANOBD2 display: receives live values from the sender via ESP-NOW and shows
// them on the LilyGO T-Display S3.
#include "DisplayApp.h"

#include <Arduino.h>
#include <WiFi.h>

#include "BootValidation.h"
#include "SystemHealth.h"
#include "DeviceSecrets.h"
#include "DiagnosticLog.h"
#include "DisplayReceiver.h"
#include "DisplaySimulation.h"
#include "DisplayUi.h"
#include "DisplayWeb.h"
#include "RuntimeSimulation.h"
#include "config/ProjectConfig.h"

namespace DisplayApp {

void begin() {
  Simulation::RuntimeSimulation::resetForBoot();
  Serial.begin(ProjectConfig::Baudrate);
  DisplayUi::begin();
  DiagnosticLog::begin();
  Security::begin(false);
  SystemHealth::logBootSummary("display");
  DiagnosticLog::appendf("[BOOT] Display firmware=%s protocol=%u mac=%s secrets=%s %s", ProjectConfig::FirmwareVersion,
                         ProjectConfig::ProtocolVersion, SystemHealth::macAddress(),
                         Security::sourceName(Security::source()), Security::problemText());
  // Web first: it sets up the AP on the ESP-NOW channel, then ESP-NOW joins.
  DisplayWeb::begin();
  DisplayReceiver::begin();
  SystemHealth::startWatchdog();
}

void tick() {
  const uint32_t now = millis();
  DisplayWeb::handle();
  DisplayReceiver::processQueuedPackets();
  DisplaySimulation::update();
  DisplayUi::handleButton();
  DisplayUi::renderIfDue();
  BootValidation::tick(now);
}

} // namespace DisplayApp
