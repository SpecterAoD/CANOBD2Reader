#include "SenderCanAlerts.h"

#include "driver/twai.h"

#include "CANHandler.h"
#include "EventThrottle.h"
#include "Logger.h"

namespace SenderCanAlerts {

namespace {
// Without a vehicle (bench) or with a wiring fault these alerts fire many
// times per second. Report at most one summary per interval so the flash log
// and the radio link are not flooded.
constexpr uint32_t AlertSummaryIntervalMs = 5000;
Logging::EventThrottle errorPassiveThrottle(AlertSummaryIntervalMs);
Logging::EventThrottle busErrorThrottle(AlertSummaryIntervalMs);
Logging::EventThrottle queueFullThrottle(AlertSummaryIntervalMs);

void report(Logging::EventThrottle& throttle, const char* text, const char* statusValue, const char* status,
            SenderCallbacks::SendStatus sendStatus) {
    if (!throttle.record(millis())) return;
    char line[80];
    snprintf(line, sizeof(line), "%s (x%lu in %lus)", text, static_cast<unsigned long>(throttle.count()),
             static_cast<unsigned long>(AlertSummaryIntervalMs / 1000));
    Logger::alarm(line);
    if (sendStatus != nullptr) sendStatus("CAN", statusValue, status);
}
} // namespace

Result process(uint32_t waitMs, SenderCallbacks::SendStatus sendStatus) {
    Result result{};
    uint32_t alertsTriggered = 0;
    twai_read_alerts(&alertsTriggered, pdMS_TO_TICKS(waitMs));

    if (alertsTriggered & TWAI_ALERT_RX_DATA) {
        CANHandler::processIncoming();
        result.rxData = true;
    }

    if (alertsTriggered & TWAI_ALERT_ERR_PASS) {
        result.errorText = "CAN error passive";
        result.errorLedRequested = true;
        report(errorPassiveThrottle, "[CAN] Error passive", "ERROR_PASSIVE", "WARN", sendStatus);
    }

    if (alertsTriggered & TWAI_ALERT_BUS_ERROR) {
        result.errorText = "CAN bus error";
        result.errorLedRequested = true;
        report(busErrorThrottle, "[CAN] Bus error", "BUS_ERROR", "ERROR", sendStatus);
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
        result.errorText = "CAN RX queue full";
        result.errorLedRequested = true;
        report(queueFullThrottle, "[CAN] RX queue full", "RX_QUEUE_FULL", "ERROR", sendStatus);
    }

    return result;
}

} // namespace SenderCanAlerts
