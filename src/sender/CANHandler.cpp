#include "CANHandler.h"
#include "config/SenderConfig.h"
#include "Utils.h"
#include "CANDecoder.h"

bool CANHandler::driverInstalled = false;
bool CANHandler::driverStarted = false;

namespace {
CanRouting::CanFrame toRoutedFrame(const twai_message_t& message) {
    CanRouting::CanFrame frame{};
    frame.id = message.identifier;
    frame.length = message.data_length_code > 8 ? 8 : message.data_length_code;
    frame.timestampMs = millis();
    for (uint8_t index = 0; index < frame.length; ++index) {
        frame.data[index] = message.data[index];
    }
    return frame;
}
} // namespace

bool CANHandler::init() {
    Logger::debug(" CAN...............INIT");
    Logger::debug(" TWAI Modus: ");
    Logger::debug("NORMAL (Senden & Empfangen)");

    // --- Treiber installieren ---
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)SenderConfig::CanTxPin,
        (gpio_num_t)SenderConfig::CanRxPin,
        TWAI_MODE_NORMAL
    );
    g_config.rx_queue_len = SenderConfig::CanRxQueueLength;
    g_config.tx_queue_len = SenderConfig::CanTxQueueLength;

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
    std::size_t processed = 0;
    while (processed < SenderConfig::MaxCanFramesPerTick &&
           twai_receive(&message, 0) == ESP_OK) {
        handleMessage(message);
        ++processed;
    }
}

void CANHandler::handleMessage(twai_message_t& message) {
    const CanRouting::CanFrame routedFrame = toRoutedFrame(message);
    CanRouting::routeFrame(routedFrame);

    // Raw CAN traffic can be extremely busy on a real vehicle. Keep it behind
    // the explicit sender flag so OBD polling and heartbeat telemetry do not
    // get starved by unrelated bus frames.
    if (SenderConfig::SendRawData) {
        CANDecoder::DecodedFrame decoded = CANDecoder::decode(message);
        Utils::sendTelemetry("CAN", "RAW", "LastCAN", decoded.raw, "", "OK");
        Utils::sendTelemetry("CAN", "HINT", "CANHint", decoded.hint, "", "OK");
        Logger::canFrame(message);
    }
}

bool CANHandler::registerListener(CanRouting::CanFrameListener& listener) {
    return CanRouting::registerListener(listener);
}

bool CANHandler::unregisterListener(CanRouting::CanFrameListener& listener) {
    return CanRouting::unregisterListener(listener);
}

void CANHandler::printStatus() {
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        const char* stateText = "UNKNOWN";
        switch (status.state) {
            case TWAI_STATE_STOPPED:    stateText = "STOPPED"; break;
            case TWAI_STATE_RUNNING:    stateText = "RUNNING"; break;
            case TWAI_STATE_BUS_OFF:    stateText = "BUS_OFF"; break;
            case TWAI_STATE_RECOVERING: stateText = "RECOVERING"; break;
            default:                    stateText = "UNKNOWN"; break;
        }
        char line[160];
        snprintf(line, sizeof(line),
                 "[TWAI] state=%s tx_failed=%lu rx_missed=%lu bus_errors=%lu arbitration_lost=%lu",
                 stateText,
                 static_cast<unsigned long>(status.tx_failed_count),
                 static_cast<unsigned long>(status.rx_missed_count),
                 static_cast<unsigned long>(status.bus_error_count),
                 static_cast<unsigned long>(status.arb_lost_count));
        Logger::twai(line);
    }
}
