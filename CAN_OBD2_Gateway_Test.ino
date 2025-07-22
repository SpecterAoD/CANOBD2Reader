/*  ============================================================================
	 CAN_OBD2_Gateway
	 Version: 0.0.1 
	 
	 Projekt basiert auf MrDIY.ca CAN Project Version: 3.1203

============================================================================= */
// ================================
// CAN_OBD2_Gateway.ino (Main File)
// ================================

#include "config.h"
#include "utils.h"
#include "esp_now_handler.h"
#include "CAN_Handler.h"
#include "OBD2_Handler.h"

#ifdef PRINT_CAN_FLAG
#include "DebugUtils.h"
#endif


void setup() {
  #ifdef DEBUG_FLAG
    Serial.begin(BUDRATE);
  #endif

  pinMode(SHIELD_LED_PIN, OUTPUT);
  digitalWrite(SHIELD_LED_PIN, HIGH);

  debugln("");
  debugln("------------------------");
  debugln("    SpecterAoD CAN OBD2 Gateway ");
  debugln("------------------------");

  currentMillis = millis();
  last_can_msg_timestamp = currentMillis - CAN_IDLE_TIMEOUT + 5;
  
  //reduceHeat();  // Optional

  if (!initESPNow() || (ENABLE_CAN && !initCAN())) {
    debugln(" SYSTEM......... FAIL");
    delay(60000);
    ESP.restart();
  }

  digitalWrite(SHIELD_LED_PIN, HIGH);
}

void loop() {
  currentMillis = millis();
  update_car_status();
  handleCANAlerts();
  handleOBD2();
  handleSleep();
}
