#include "SenderTelemetry.h"

#include <string.h>

#include <atomic>

#include "EspNowTelemetryTransport.h"
#include "EventThrottle.h"
#include "LinkBackoff.h"
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

// Link state from the ESP-NOW delivery callback (WiFi task -> loop).
std::atomic<uint32_t> deliveredCount{0};
std::atomic<uint32_t> undeliveredCount{0};
std::atomic<bool> lastDeliveryOk{false};
uint32_t seenDelivered = 0;
uint32_t seenUndelivered = 0;
// Display off / out of range: after 5 failures only one probe per second.
Transport::LinkBackoff linkBackoff(5, 1000);
Logging::EventThrottle failureLog(10000);
bool linkWasOffline = false;

// Feeds delivery results into the backoff and logs (throttled) state changes.
void updateLinkState(uint32_t nowMs) {
    const uint32_t delivered = deliveredCount.load();
    const uint32_t undelivered = undeliveredCount.load();
    for (uint32_t i = seenUndelivered; i != undelivered; ++i) linkBackoff.reportResult(false);
    if (delivered != seenDelivered && lastDeliveryOk.load()) linkBackoff.reportResult(true);
    seenDelivered = delivered;
    seenUndelivered = undelivered;

    if (linkBackoff.offline() != linkWasOffline) {
        linkWasOffline = linkBackoff.offline();
        WebConsoleHandler::log(linkWasOffline ? "[ESP-NOW] Display antwortet nicht - sende nur noch 1x/s"
                                              : "[ESP-NOW] Display wieder erreichbar");
    }
    (void)nowMs;
}

void noteSendFailure(esp_err_t result, uint32_t nowMs) {
    ++sendFail;
    linkBackoff.reportResult(false);
    lastSendErrorText = "ESP-NOW Sendefehler " + String(static_cast<int>(result));
    lastErrorText = lastSendErrorText;
    if (failureLog.record(nowMs)) {
        WebConsoleHandler::log("[ESP-NOW] Sendefehler err=" + String(static_cast<int>(result)) + " (x" +
                               String(static_cast<unsigned long>(failureLog.count())) + " in 10 s)");
    }
}

// Common gate for every packet: false = skipped because the link is offline.
bool mayTransmit(uint32_t nowMs) {
    updateLinkState(nowMs);
    return linkBackoff.allowSend(nowMs);
}

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
    // Gate first: skipped packets must not consume sequence numbers, otherwise
    // the display counts them as lost.
    const uint32_t now = millis();
    if (!mayTransmit(now)) return;

    TelemetryProtocol::buildPayload(payload, sizeof(payload),
                                    type, key, name, value, unit, status,
                                    currentSequence = Telemetry::nextSequence());
    WebConsoleHandler::recordTelemetry(payload);

    Telemetry::TelemetryCodec::encodeText(frame, packetTypeFor(type), currentSequence, now, payload);
    const esp_err_t sendResult = Transport::sendTelemetryPacket(frame);
    if (sendResult == ESP_OK) {
        ++sendOk;
        lastSendErrorText = "";
    } else {
        noteSendFailure(sendResult, now);
    }

    maybeLogTelemetrySendSummary();

    if (status != nullptr && strcmp(status, "OK") != 0) {
        lastErrorText = String(type == nullptr ? "" : type) + "/" + (key == nullptr ? "" : key) + ": " + status;
    }
}

void sendBinary(Telemetry::PacketType type, const uint8_t* data, std::size_t length) {
    const uint32_t now = millis();
    if (!mayTransmit(now)) return;
    currentSequence = Telemetry::nextSequence();
    if (!Telemetry::TelemetryCodec::encode(frame, type, currentSequence, now, data, length)) return;
    const esp_err_t result = Transport::sendTelemetryPacket(frame);
    if (result == ESP_OK) {
        ++sendOk;
        lastSendErrorText = "";
    } else {
        noteSendFailure(result, now);
    }
}

void onDelivery(bool delivered) {
    // Runs in the WiFi task: only touch atomics here.
    lastDeliveryOk.store(delivered);
    if (delivered) {
        deliveredCount.fetch_add(1);
    } else {
        undeliveredCount.fetch_add(1);
    }
}

bool linkOffline() {
    return linkBackoff.offline();
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
