#include "OBDHandler.h"
#include "Utils.h"
#include "PID_Converter.h"

bool OBD2Handler::sendRequest(uint8_t mode, uint8_t pid) {
    twai_message_t request = {};
    request.identifier = 0x7DF;
    request.extd = 0;
    request.rtr = 0;
    request.data_length_code = 8;
    request.data[0] = 0x02;
    request.data[1] = mode;
    request.data[2] = pid;
    for (int i = 3; i < 8; i++) request.data[i] = 0x55;

    esp_err_t result = twai_transmit(&request, pdMS_TO_TICKS(10));
    return result == ESP_OK;
}

bool OBD2Handler::receiveResponse(uint8_t mode, uint8_t pid, uint8_t* outData, uint8_t& outLen) {
    unsigned long start = millis();
    while (millis() - start < 200) {
        twai_message_t response;
        if (twai_receive(&response, pdMS_TO_TICKS(10)) == ESP_OK) {
            if (response.identifier >= 0x7E8 && response.identifier <= 0x7EF) {
                if (response.data_length_code >= 3 &&
                    response.data[1] == (mode | 0x40) &&
                    response.data[2] == pid) {
                    outLen = response.data_length_code - 3;
                    if (outLen > 5) outLen = 5; // OBD2 payload: max 5 bytes (8 CAN bytes - 3 header bytes)
                    memcpy(outData, &response.data[3], outLen);
                    return true;
                }
            }
        }
    }
    return false;
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
