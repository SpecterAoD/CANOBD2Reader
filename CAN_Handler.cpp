// === CAN_Handler.cpp ===
#include "CAN_Handler.h"
#include "config.h"

bool initCAN() {
  debugln(" CAN...............INIT");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)SHIELD_CAN_TX, (gpio_num_t)SHIELD_CAN_RX, TWAI_OPERATION_MODE);
  g_config.rx_queue_len = 32;
  g_config.tx_queue_len = 2;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    debugln(" CAN....Driver....FAIL");
    return false;
  }

  if (twai_start() != ESP_OK) {
    debugln(" CAN....Start......FAIL");
    return false;
  }

  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK) {
    debugln(" CAN....Alerts.....FAIL");
    return false;
  }

  debugln(" CAN.................OK");
  return true;
}

void handleCANAlerts() {
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));

  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      processCANMessage(message);
    }
    last_can_msg_timestamp = millis();
  }

  if (alerts_triggered & TWAI_ALERT_ERR_PASS) debugln(" CAN MSG..........ERROR");
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) debugln(" CAN MSG......BUS ERROR");
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) debugln(" CAN MSG.........Q FULL");
}

void processCANMessage(twai_message_t &message) {
  esp_now_frame_t frame;
  frame.mesh_id = esp_now_mesh_id;
  frame.can_id = message.identifier;
  frame.len = message.data_length_code;
  memcpy(frame.d, message.data, message.data_length_code);

  esp_now_send(broadcastAddress, (uint8_t*)&frame, sizeof(frame));
  debug("CAN RX 0x"); debug(message.identifier, HEX);
  debug(" ["); debug(message.data_length_code); debug("] <");
  for (int i = 0; i < message.data_length_code; i++) {
    if (i > 0) debug(":");
    debug(message.data[i], HEX);
  }
  debugln("> (gesendet)");
}