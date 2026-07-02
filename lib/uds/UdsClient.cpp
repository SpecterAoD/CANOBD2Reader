#include "UdsClient.h"

#if defined(ARDUINO)
#include "Logger.h"
#include "SenderConfig.h"
#endif

#include "UdsDiagnostics.h"

namespace Uds {

bool Client::isServiceAllowed(Service service) {
    switch (service) {
        case Service::DiagnosticSessionControl:
        case Service::ReadDTCInformation:
        case Service::ReadDataByIdentifier:
        case Service::TesterPresent:
            return true;
        case Service::ECUReset:
        case Service::SecurityAccess:
            return false;
    }
    return false;
}

bool Client::requestDefaultSession(Response& response) {
    const uint8_t payload[] = {serviceId(Service::DiagnosticSessionControl), 0x01};
    return request(Service::DiagnosticSessionControl, payload, sizeof(payload), response);
}

bool Client::testerPresent(Response& response) {
    const uint8_t payload[] = {serviceId(Service::TesterPresent), 0x00};
    return request(Service::TesterPresent, payload, sizeof(payload), response);
}

bool Client::readDataByIdentifier(uint16_t did, Response& response) {
    const uint8_t payload[] = {
        serviceId(Service::ReadDataByIdentifier),
        static_cast<uint8_t>((did >> 8U) & 0xFFU),
        static_cast<uint8_t>(did & 0xFFU)
    };
    return request(Service::ReadDataByIdentifier, payload, sizeof(payload), response);
}

bool Client::readDtcInformation(uint8_t subFunction, uint8_t statusMask, Response& response) {
    const uint8_t payload[] = {serviceId(Service::ReadDTCInformation), subFunction, statusMask};
    return request(Service::ReadDTCInformation, payload, sizeof(payload), response);
}

bool Client::request(Service service,
                     const uint8_t* payload,
                     std::size_t payloadLength,
                     Response& response) {
    response = Response{};
    response.service = service;

    if (!isServiceAllowed(service)) {
#if defined(ARDUINO)
        Logger::warn("[UDS] Blocked unsafe service");
#endif
        return false;
    }

    Diagnostics::recordRequest(service, payload, payloadLength, isoTp_.requestId());

    if (!isoTp_.sendRequest(serviceId(service), payload, payloadLength)) {
        Diagnostics::recordSendFailure();
        return false;
    }

    IsoTp::Payload payloadResponse{};
#if defined(ARDUINO)
    const uint32_t timeoutMs = SenderConfig::UdsResponseTimeoutMs;
#else
    const uint32_t timeoutMs = 250;
#endif
    if (!isoTp_.receiveResponse(serviceId(service), 0xFF, payloadResponse, timeoutMs)) {
        if (isoTp_.lastResponseWasNegative()) {
            response.negative = true;
            response.negativeService = isoTp_.lastNegativeService();
            response.negativeCode = isoTp_.lastNegativeCode();
            Diagnostics::recordNegativeResponse(response.negativeService, response.negativeCode);
        } else {
            Diagnostics::recordTimeout();
        }
        return false;
    }

    if (payloadResponse.length == 0 ||
        payloadResponse.bytes[0] != positiveResponseId(service)) {
        Diagnostics::recordTimeout();
        return false;
    }

    response.positive = true;
    response.responseId = payloadResponse.responseId;
    response.data = payloadResponse.bytes.data();
    response.length = payloadResponse.length;
    Diagnostics::recordPositiveResponse(response.responseId, response.data, response.length);
    return true;
}

} // namespace Uds
