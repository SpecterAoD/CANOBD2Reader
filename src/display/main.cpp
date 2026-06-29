#include <Arduino.h>
#include "DisplayApp.h"

void setup() {
  DisplayApp::begin();
}

void loop() {
  DisplayApp::tick();
}
