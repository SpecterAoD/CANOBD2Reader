#include <Arduino.h>
#include "DisplayApp.h"
#include "DisplayReceiver.h"
#include "DisplaySimulation.h"
#include "DisplayOta.h"
#include "DisplayUi.h"

namespace DisplayApp {
  void begin() {
    Serial.begin(115200);
    DisplayUi::begin();
    DisplayOta::begin();
    DisplayReceiver::begin();
  }

  void tick() {
    DisplayOta::handle();
    DisplaySimulation::update();
    DisplayUi::handleButton();
    DisplayUi::renderIfDue();
  }
}
