// OBD2_Handler.h – Abfragen und Senden von OBD2-Daten
#ifndef OBD2_HANDLER_H
#define OBD2_HANDLER_H

#include <Arduino.h>
#include "driver/twai.h"
#include "config.h"
#include "PID_Converter.h"
#include "esp_now_handler.h"

// === Funktionen ===
void handleOBD2();
bool sendOBDRequest(byte mode, byte pid);
bool receiveOBDResponse(byte mode, byte pid, byte* outData, byte& outLen);
void requestAndSendOBDPID(byte pid);

#endif
