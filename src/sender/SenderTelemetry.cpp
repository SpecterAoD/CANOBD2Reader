#include "SenderTelemetry.h"

#include <string.h>

#include "EspNowTelemetryTransport.h"
#include "RuntimeSimulation.h"
#include "TelemetryCodec.h"
#include "TelemetryProtocol.h"
#include "TelemetrySequence.h"
#include "WebConsoleHandler.h"
#include "config/LoggingConfig.h"
#include "config/ProjectConfig.h"

namespace {
Telemetry::TelemetryPacket frame{};
char payload[ProjectConfig::TelemetryPayloadSize] = {};
uint32_t currentSequence = 0;
uint32_t sendOk = 0;
uint32_t sendFail = 0;
uint32_t lastSummaryAt = 0;
uint32_t lastObdResponse = 0;
String lastErrorText;
String lastSendErrorText;

Telemetry::PacketType packetTypeFor(const char* type) {
    if (type == nullptr) return Telemetry::PacketType::Text;
    if (strcmp(type, "OBD") == 0) return Telemetry::PacketType::Obd;
    if (strcmp(type, "CAN") == 0) return Telemetry::PacketType::Can;
    if (strcmp(type, "DTC") == 0) return Telemetry::PacketType::Diagnostic;
    if (strcmp(type, "STATUS") == 0) return Telemetry::PacketType::Status;
    return Telemetry::PacketType::Text;
}

void maybeLogTelemetrySendSummary() {
    if (!(LoggingConfig::TraceSenderTelemetry && LoggingConfig::SerialEnabled)) return;

    const uint32_t now = millis();
    if (lastSummaryAt == 0) {
        lastSummaryAt = now;
        return;
    }

    if (now - lastSummaryAt < LoggingConfig::TraceSummaryIntervalMs) return;

    WebConsoleHandler::log("[sender-tx] seq=" + String(static_cast<unsigned long>(currentSequence)) +
                           " ok=" + String(static_cast<unsigned long>(sendOk)) +
                           " fail=" + String(static_cast<unsigned long>(sendFail)) +
                           " sim=" + String(Simulation::RuntimeSimulation::enabled() ? "on" : "off"));
    lastSummaryAt = now;
}
} // namespace

namespace SenderTelemetry {

void reset() {
    frame = Telemetry::TelemetryPacket{};
    payload[0] = '\0';
    currentSequence = 0;
    sendOk = 0;
    sendFail = 0;
    lastSummaryAt = 0;
    lastObdResponse = 0;
    lastErrorText = "";
    lastSendErrorText = "";
}

void send(const char* type,
          const char* key,
          const char* name,
          const char* value,
          const char* unit,
          const char* status) {
    TelemetryProtocol::buildPayload(payload, sizeof(payload),
                                    type, key, name, value, unit, status,
                                    currentSequence = Telemetry::nextSequence());

    Telemetry::TelemetryCodec::encodeText(frame,
                                          packetTypeFor(type),
                                          currentSequence,
                                          millis(),
                                          payload);
    const esp_err_t sendResult = Transport::sendTelemetryPacket(frame);
    if (sendResult == ESP_OK) {
        ++sendOk;
        lastSendErrorText = "";
        if (strcmp(type, "STATUS") == 0 && strcmp(key, "HEARTBEAT") == 0) {
            WebConsoleHandler::log("[ESP-NOW] Heartbeat sent seq=" +
                                   String(static_cast<unsigned long>(currentSequence)));
        }
    } else {
        ++sendFail;
        lastSendErrorText = "ESP-NOW send error " + String(static_cast<int>(sendResult));
        lastErrorText = lastSendErrorText;
        WebConsoleHandler::log("[ESP-NOW] Send failed error=" + String(static_cast<int>(sendResult)) +
                               " seq=" + String(static_cast<unsigned long>(currentSequence)));
        if (LoggingConfig::TraceSenderTelemetry && LoggingConfig::SerialEnabled) {
            WebConsoleHandler::log("[sender-tx] send failed seq=" +
                                   String(static_cast<unsigned long>(currentSequence)) +
                                   " err=" + String(static_cast<int>(sendResult)));
        }
    }

    maybeLogTelemetrySendSummary();
    WebConsoleHandler::recordTelemetry(payload);

    if (status != nullptr && strcmp(status, "OK") != 0) {
        lastErrorText = String(type == nullptr ? "" : type) + "/" + (key == nullptr ? "" : key) + ": " + status;
    }
    if (type != nullptr && status != nullptr && strcmp(type, "OBD") == 0 && strcmp(status, "OK") == 0) {
        lastObdResponse = millis();
    }
}

void sendStatus(const char* key, const char* value, const char* status) {
    send("STATUS", key, key, value, "", status);
}

uint32_t sequence() {
    return currentSequence;
}

uint32_t sendOkCount() {
    return sendOk;
}

uint32_t sendFailCount() {
    return sendFail;
}

uint32_t lastObdResponseAt() {
    return lastObdResponse;
}

uint32_t& lastObdResponseAtRef() {
    return lastObdResponse;
}

const char* lastPayload() {
    return payload;
}

const String& lastError() {
    return lastErrorText;
}

const String& lastSendError() {
    return lastSendErrorText;
}

void setLastError(const String& error) {
    lastErrorText = error;
}

} // namespace SenderTelemetry
