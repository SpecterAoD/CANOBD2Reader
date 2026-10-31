#include "SenderCanAlerts.h"

#include "driver/twai.h"

#include "CANHandler.h"
#include "Logger.h"

namespace SenderCanAlerts {

Result process(uint32_t waitMs, SenderCallbacks::SendStatus sendStatus) {
    Result result{};
    uint32_t alertsTriggered = 0;
    twai_read_alerts(&alertsTriggered, pdMS_TO_TICKS(waitMs));

    if (alertsTriggered & TWAI_ALERT_RX_DATA) {
        CANHandler::processIncoming();
        result.rxData = true;
    }

    if (alertsTriggered & TWAI_ALERT_ERR_PASS) {
        Logger::debug("[CAN] TWAI controller is error passive");
        result.errorText = "CAN error passive";
        result.errorLedRequested = true;
        if (sendStatus != nullptr) sendStatus("CAN", "ERROR_PASSIVE", "WARN");
    }

    if (alertsTriggered & TWAI_ALERT_BUS_ERROR) {
        Logger::alarm("[CAN] Bus error");
        result.errorText = "CAN bus error";
        result.errorLedRequested = true;
        if (sendStatus != nullptr) sendStatus("CAN", "BUS_ERROR", "ERROR");
    }

    // Bus-off (e.g. wiring fault, wrong bitrate, ignition glitch): the TWAI
    // controller stops completely until recovery is initiated. Recover
    // automatically instead of waiting for a reboot.
    if (alertsTriggered & TWAI_ALERT_BUS_OFF) {
        Logger::alarm("[CAN] Bus-Off, starte Recovery");
        result.errorText = "CAN bus-off";
        result.errorLedRequested = true;
        if (sendStatus != nullptr) sendStatus("CAN", "BUS_OFF", "ERROR");
        twai_initiate_recovery();
    }

    if (alertsTriggered & TWAI_ALERT_BUS_RECOVERED) {
        // After recovery the controller is in STOPPED state and must be restarted.
        const bool restarted = twai_start() == ESP_OK;
        Logger::alarm(restarted ? "[CAN] Bus recovered, TWAI neu gestartet" : "[CAN] TWAI Neustart fehlgeschlagen");
        if (sendStatus != nullptr) sendStatus("CAN", restarted ? "RECOVERED" : "RESTART_FAIL", restarted ? "OK" : "ERROR");
    }

    if (alertsTriggered & TWAI_ALERT_RX_QUEUE_FULL) {
        Logger::alarm("[CAN] RX queue full");
        result.errorText = "CAN RX queue full";
        result.errorLedRequested = true;
        if (sendStatus != nullptr) sendStatus("CAN", "RX_QUEUE_FULL", "ERROR");
    }

    return result;
}

} // namespace SenderCanAlerts
