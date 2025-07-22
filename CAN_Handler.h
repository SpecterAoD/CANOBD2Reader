// CAN_Handler.h – Initialisierung und Empfang von CAN-Nachrichten
#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include <Arduino.h>
#include "driver/twai.h"
#include "config.h"
#include "esp_now_handler.h"
#include "utils.h"

// === Funktionen ===
bool initCAN();
void handleCANAlerts();
void processCANMessage(twai_message_t &message);

#endif

