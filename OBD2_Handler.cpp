// OBD2_Handler.cpp – Abfragen und Senden von OBD2-Daten
#include "OBD2_Handler.h"

#include "config.h"
#include "PIDs.h"

void handleOBD2() {
  static unsigned long last_obd_request_time = 0;

  if (!ENABLE_OBD2 || car_status != CAR_IS_RUNNING) return;

  if (millis() - last_obd_request_time >= OBD_INTERVAL_MS) {
    for (byte i = 0; i < NUM_OBD_PIDS; i++) {
      requestAndSendOBDPID(OBD_PIDS_TO_QUERY[i]);
    }
    last_obd_request_time = millis();
  }
}


bool sendOBDRequest(byte mode, byte pid) {
  twai_message_t request = {};
  request.identifier = 0x7DF;
  request.extd = 0;
  request.rtr = 0;
  request.data_length_code = 8;
  request.data[0] = 0x02; // Länge des Dateninhalts
  request.data[1] = mode; // Mode (0x01 = Live Data)
  request.data[2] = pid;  // Angefragter PID
  for (int i = 3; i < 8; i++) request.data[i] = 0x55;

  debug("OBD2 Anfrage: MODE=0x");
  debug(mode, HEX);
  debug(" PID=");
  debugln(pid, HEX);

  return twai_transmit(&request, pdMS_TO_TICKS(10)) == ESP_OK;
}

bool receiveOBDResponse(byte mode, byte pid, byte* outData, byte& outLen) {
  unsigned long start = millis();
  while (millis() - start < 200) {
    twai_message_t response;
    if (twai_receive(&response, pdMS_TO_TICKS(10)) == ESP_OK) {
      if (response.identifier >= 0x7E8 && response.identifier <= 0x7EF) {
        if (response.data[1] == (mode | 0x40) && response.data[2] == pid) {
          outLen = response.data_length_code - 3;
          memcpy(outData, &response.data[3], outLen);

          debug("OBD2 Antwort empfangen: ID=0x");
          debugln(response.identifier, HEX);
		  for (byte i = 0; i < outLen; i++) {
            if (outData[i] < 0x10) debug("0");
            debug(outData[i], HEX);
            debug(" ");
          }
          debugln("");
          return true;
        }
      }
    }
  }
  debugln("OBD2 Antwort: TIMEOUT");
  return false;
}

void requestAndSendOBDPID(byte pid) {
  if (!sendOBDRequest(read_LiveData, pid)) {
    debugln("OBD2 Anfrage FEHLGESCHLAGEN");
    return;
  }

  if (receiveOBDResponse(read_LiveData, pid, responseData, responseLen)) {
    PIDResult result = convertPID(pid, responseData, responseLen);

    debug("← OBD PID 0x");
    debug(pid, HEX);
    debug(" [");
    debug(getPIDName(pid));
    debug("] = ");
    debug(result.value);
    debug(" ");
    debugln(result.unit);

    snprintf(text_frame.payload, 128, "%02X,%s,%.2f,%s", pid, getPIDName(pid), result.value, result.unit);
    esp_now_send(broadcastAddress, (uint8_t*)&text_frame, sizeof(text_frame));
  } else {
    debug("← OBD PID 0x");
    debug(pid, HEX);
    debug(" [");
    debug(getPIDName(pid));
    debugln("] TIMEOUT");

    snprintf(text_frame.payload, 128, "%02X,%s,ERROR,N/A", pid, getPIDName(pid));
    esp_now_send(broadcastAddress, (uint8_t*)&text_frame, sizeof(text_frame));
  }
