#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

#include <Arduino.h>
#include "driver/twai.h"
#include "esp_now_handler.h"  // Für esp_now_frame_t

#ifdef PRINT_CAN_FLAG

// CAN-Nachricht ausgeben
void printFrame(const twai_message_t& message) {
  Serial.print(message.extd ? "CAX: 0x" : "CAN: 0x");
  Serial.print(message.identifier, HEX);
  Serial.print(" (");
  Serial.print(message.identifier, DEC);
  Serial.print(")");
  Serial.print(" [");
  Serial.print(message.data_length_code);
  Serial.print("] <");

  for (int i = 0; i < message.data_length_code; i++) {
    if (i != 0) Serial.print(":");
    Serial.print(message.data[i], HEX);
  }
  Serial.println(">");
}

// ESP-NOW-Datenrahmen anzeigen
void printCAN(const esp_now_text_frame_t& message) {
  Serial.print("ESP ");
  Serial.print(message.mesh_id);
  Serial.print(":  0x");
  Serial.print(message.can_id, HEX);
  Serial.print(" (");
  Serial.print(message.can_id, DEC);
  Serial.print(")");
  Serial.print(" [");
  Serial.print(message.len);
  Serial.print("] <");

  for (int i = 0; i < message.len; i++) {
    if (i != 0) Serial.print(":");
    Serial.print(message.d[i], HEX);
  }
  Serial.println(">");
}

#endif // PRINT_CAN_FLAG

#endif // DEBUGUTILS_H
