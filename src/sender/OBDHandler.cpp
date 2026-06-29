#include "OBDHandler.h"
#include "Utils.h"
#include "PID_Converter.h"
#include "IsoTpHandler.h"

namespace {
IsoTp::IsoTpHandler isoTp;
}

bool OBD2Handler::sendRequest(uint8_t mode, uint8_t pid) {
    return isoTp.sendRequest(mode, pid);
}

bool OBD2Handler::receiveResponse(uint8_t mode, uint8_t pid, uint8_t* outData, uint8_t& outLen) {
    IsoTp::Payload response{};
    if (!isoTp.receiveResponse(mode, pid, response, Config::Sender::ObdResponseTimeoutMs)) return false;
    if (response.length < 2) return false;

    const size_t payloadOffset = 2; // positive response mode + PID
    const size_t available = response.length - payloadOffset;
    outLen = static_cast<uint8_t>(available > 8 ? 8 : available);
    memcpy(outData, response.bytes.data() + payloadOffset, outLen);
    return true;
}

void OBD2Handler::calcConsumption(uint8_t pid, float value) {
    if (pid == VEHICLE_SPEED) Config::lastSpeed = value;
    if (pid == ENGINE_FUEL_RATE) Config::lastFuelRate = value;

    if (Config::lastSpeed < 1.0f || Config::lastFuelRate < 0.1f) return;

    Config::consumption = (Config::lastFuelRate / Config::lastSpeed) * 100.0f;
    Config::consumptionSum += Config::consumption;
    Config::consumptionCount++;

    if (Config::consumptionCount >= 10) {
        float avg = Config::consumptionSum / Config::consumptionCount;
        char avgText[16];
        snprintf(avgText, sizeof(avgText), "%.2f", avg);
        Utils::sendTelemetry("FUEL", "AVG", "AverageConsumption", avgText, "L/100km", "OK");

        Config::consumptionSum = 0.0f;
        Config::consumptionCount = 0;
    }
}

void OBD2Handler::requestAndSendPID(uint8_t pid) {
    if (!sendRequest(read_LiveData, pid)) {
        Utils::sendOBDError(pid, getPIDName(pid));
        return;
    }

    uint8_t responseData[8];
    uint8_t responseLen = 0;

    if (receiveResponse(read_LiveData, pid, responseData, responseLen)) {
        if constexpr (Config::Sender::SendRawDataOnly) {
            Utils::sendRawOBDData(pid, responseData, responseLen);
        } else {
            PIDResult result = convertPID(pid, responseData, responseLen);

            if (pid == VEHICLE_SPEED || pid == ENGINE_FUEL_RATE) {
                calcConsumption(pid, result.value);
            }

            Utils::sendOBDValue(pid, getPIDName(pid), result.value, result.unit);
        }
    } else {
        Utils::sendOBDError(pid, getPIDName(pid));
    }
}
