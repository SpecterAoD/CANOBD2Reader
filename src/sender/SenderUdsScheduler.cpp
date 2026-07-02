#include "SenderUdsScheduler.h"

#include "config/SenderConfig.h"
#include "UdsClient.h"
#include "UdsDecoder.h"
#include "UdsDiagnostics.h"
#include "IsoTpTypes.h"
#include "WebConsoleHandler.h"

namespace {
constexpr uint16_t kUdsVinDid = 0xF190;

Uds::Client udsClient;
uint32_t lastQueryAt = 0;
String didText = "--";
String dtcText = "--";
} // namespace

namespace SenderUdsScheduler {

void reset() {
    lastQueryAt = 0;
    didText = "--";
    dtcText = "--";
}

void tick(uint32_t nowMs,
          uint32_t& lastCanMessageAt,
          uint32_t& lastObdResponseAt,
          bool& canBusActive,
          SenderCallbacks::SendTelemetry sendTelemetry,
          SenderCallbacks::SendStatus sendStatus) {
    if (!SenderConfig::EnableUDS) return;
    if (nowMs - lastQueryAt < SenderConfig::UdsQueryIntervalMs) return;
    lastQueryAt = nowMs;

    udsClient.setRequestId(IsoTp::PhysicalRequestId);

    Uds::Response response{};
    if (udsClient.testerPresent(response)) {
        if (sendStatus != nullptr) sendStatus("UDS", "TESTER_PRESENT", "OK");
        lastCanMessageAt = nowMs;
        lastObdResponseAt = nowMs;
        canBusActive = true;
    } else if (sendStatus != nullptr) {
        sendStatus("UDS", response.negative ? Uds::Diagnostics::lastNegativeResponse() : "NO_RESPONSE",
                   response.negative ? "ERROR" : "WARN");
    }

    if (udsClient.readDataByIdentifier(kUdsVinDid, response)) {
        char vin[24] = {};
        if (Uds::decodeAsciiDid(response.data, response.length, kUdsVinDid, vin, sizeof(vin))) {
            didText = String("VIN=") + vin;
            Uds::Diagnostics::setLastDid(kUdsVinDid, vin);
            WebConsoleHandler::log("[UDS] DID 0xF190 VIN=" + String(vin));
            if (sendTelemetry != nullptr) sendTelemetry("STATUS", "UDS_DID_F190", "UDS_VIN", vin, "", "OK");
            lastCanMessageAt = nowMs;
            lastObdResponseAt = nowMs;
            canBusActive = true;
        } else {
            didText = "DID_F190_INVALID";
            Uds::Diagnostics::setLastDid(kUdsVinDid, "INVALID");
            WebConsoleHandler::log("[UDS] DID 0xF190 invalid response");
            if (sendTelemetry != nullptr) sendTelemetry("STATUS", "UDS_DID_F190", "UDS_VIN", "N/A", "", "ERROR");
        }
    } else {
        didText = response.negative ? Uds::Diagnostics::lastNegativeResponse() : "TIMEOUT";
        Uds::Diagnostics::setLastDid(kUdsVinDid, didText.c_str());
        if (sendTelemetry != nullptr) {
            sendTelemetry("STATUS", "UDS_DID_F190", "UDS_VIN", didText.c_str(), "", response.negative ? "ERROR" : "TIMEOUT");
        }
    }

    if (udsClient.readDtcInformation(0x02, 0xFF, response)) {
        char summary[48];
        snprintf(summary, sizeof(summary), "len=%u", static_cast<unsigned>(response.length));
        dtcText = summary;
        Uds::Diagnostics::setLastDtcSummary(summary);
        WebConsoleHandler::log("[UDS] DTC information " + String(summary));
        if (sendTelemetry != nullptr) sendTelemetry("DTC", "UDS", "UDS_DTC", summary, "", "OK");
        lastCanMessageAt = nowMs;
        lastObdResponseAt = nowMs;
        canBusActive = true;
    } else {
        dtcText = response.negative ? Uds::Diagnostics::lastNegativeResponse() : "TIMEOUT";
        Uds::Diagnostics::setLastDtcSummary(dtcText.c_str());
        if (sendTelemetry != nullptr) {
            sendTelemetry("DTC", "UDS", "UDS_DTC", dtcText.c_str(), "", response.negative ? "ERROR" : "TIMEOUT");
        }
    }
}

const String& lastDidText() {
    return didText;
}

const String& lastDtcText() {
    return dtcText;
}

} // namespace SenderUdsScheduler
