#include <Arduino.h>
#include "SenderApp.h"

void setup() {
  SenderApp::begin();
}

void loop() {
  SenderApp::tick();
}
