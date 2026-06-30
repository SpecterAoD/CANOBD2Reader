#include <Arduino.h>
#include "DisplayApp.h"
#include "DisplayReceiver.h"
#include "DisplaySimulation.h"
#include "DisplayOta.h"
#include "DisplayUi.h"
#include "RuntimeSimulation.h"

namespace DisplayApp {
  void begin() {
    Simulation::RuntimeSimulation::resetForBoot();
    Serial.begin(115200);
    DisplayUi::begin();
    DisplayOta::begin();
    DisplayReceiver::begin();
  }

  void tick() {
    DisplayOta::handle();
    DisplayReceiver::processQueuedPackets();
    DisplaySimulation::update();
    DisplayUi::handleButton();
    DisplayUi::renderIfDue();
  }
}
