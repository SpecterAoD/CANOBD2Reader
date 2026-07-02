#pragma once

#include <Arduino.h>
#include <driver/twai.h>

#include "IsoTpTypes.h"

class OBD2Handler {
public:
    /// Sends a Mode/PID OBD-II request through the shared ISO-TP layer.
    static bool sendRequest(uint8_t mode, uint8_t pid);

    /// Sends a generic OBD-II payload, for example Mode 03 without a PID.
    static bool sendRequestPayload(uint8_t mode, const uint8_t* payload, size_t payloadLength);

    /// Receives the data bytes for a Mode/PID response without the response mode/PID prefix.
    static bool receiveResponse(uint8_t mode, uint8_t pid, uint8_t* outData, uint8_t& outLen);

    /// Receives the complete ISO-TP payload including the positive response byte.
    static bool receivePayload(uint8_t mode, uint8_t pid, IsoTp::Payload& outPayload);

    /// Sends a generic OBD payload and waits for the matching ISO-TP response.
    static bool requestPayload(uint8_t mode,
                               const uint8_t* payload,
                               size_t payloadLength,
                               uint8_t expectedPid,
                               IsoTp::Payload& outPayload);

    /// Requests a PID, decodes it and forwards it as telemetry.
    static bool requestAndSendPID(uint8_t pid);

    /// Calculates and sends consumption from speed and fuel rate.
    static void calcConsumption(uint8_t pid, float value);

    /// Updates MAP/BARO state and sends the centrally calculated boost pressure.
    static void updateBoostPressure(uint8_t pid, float value);

    static bool lastResponseWasNegative();
    static uint8_t lastNegativeService();
    static uint8_t lastNegativeCode();
    static uint32_t activeRequestId();
};
