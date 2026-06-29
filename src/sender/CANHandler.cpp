#include "CANHandler.h"
#include "Config.h"
#include "Utils.h"
#include "CANDecoder.h"

bool CANHandler::driverInstalled = false;
bool CANHandler::driverStarted = false;

bool CANHandler::init() {
    Logger::debug(" CAN...............INIT");
    Logger::debug(" TWAI Modus: ");
    if (Config::Sender::TwaiOperationMode == TWAI_MODE_NORMAL) {
        Logger::debug("NORMAL (Senden & Empfangen)");
    } else {
        Logger::debug("LISTEN_ONLY (Nur Empfangen)");
    }

    // --- Treiber installieren ---
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)Config::Sender::ShieldCanTx,
        (gpio_num_t)Config::Sender::ShieldCanRx,
        Config::Sender::TwaiOperationMode
    );
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 2;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Logger::debug(" CAN....Driver....FAIL");
        return false;
    }
    driverInstalled = true;
    Logger::debug(" CAN....Install......OK");

    // --- Treiber starten ---
    if (twai_start() != ESP_OK) {
        Logger::debug(" CAN....Start......FAIL");
        shutdown();
        return false;
    }
    driverStarted = true;
    Logger::debug(" CAN....Start........OK");

    if (!configureAlerts()) {
        shutdown();
        return false;
    }
    return true;
}

bool CANHandler::configureAlerts() {
    uint32_t alerts = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS |
                      TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;

    if (twai_reconfigure_alerts(alerts, nullptr) != ESP_OK) {
        Logger::debug(" CAN....Alerts.....FAIL");
        return false;
    }

    Logger::debug(" CAN....Alerts.......OK");
    Logger::debug(" CAN.................OK");
    return true;
}

void CANHandler::shutdown() {
    if (driverStarted) {
        twai_stop();
        driverStarted = false;
    }
    if (driverInstalled) {
        twai_driver_uninstall();
        driverInstalled = false;
    }
}

void CANHandler::processIncoming() {
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
        handleMessage(message);
    }
}

void CANHandler::handleMessage(twai_message_t& message) {
    // Send one canonical CAN telemetry payload. The old raw duplicate was
    // removed because the display now consumes the shared TelemetryPacket format.
    CANDecoder::DecodedFrame decoded = CANDecoder::decode(message);
    Utils::sendTelemetry("CAN", "RAW", "LastCAN", decoded.raw, "", "OK");
    Utils::sendTelemetry("CAN", "HINT", "CANHint", decoded.hint, "", "OK");

    // Debug-Ausgabe
    Logger::canFrame(message);
}

void CANHandler::printStatus() {
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        Serial.println("------- TWAI Status -------");
        Serial.print("Controller State: ");
        switch (status.state) {
            case TWAI_STATE_STOPPED:    Serial.println("STOPPED"); break;
            case TWAI_STATE_RUNNING:    Serial.println("RUNNING"); break;
            case TWAI_STATE_BUS_OFF:    Serial.println("BUS OFF"); break;
            case TWAI_STATE_RECOVERING: Serial.println("RECOVERING"); break;
            default:                    Serial.println("UNKNOWN"); break;
        }
        Serial.print("TX Failed: "); Serial.println(status.tx_failed_count);
        Serial.print("RX Missed: "); Serial.println(status.rx_missed_count);
        Serial.print("Bus Errors: "); Serial.println(status.bus_error_count);
        Serial.print("Arbitration Lost: "); Serial.println(status.arb_lost_count);
        Serial.println("---------------------------");
    }
}
