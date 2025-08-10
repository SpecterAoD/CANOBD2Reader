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
                if (response.data[1] == (mode | 0x40) && response.data[2] == pid) {
                    outLen = response.data_length_code - 3;
                    memcpy(outData, &response.data[3], outLen);
                    return true;
                }
            }
        }
    }
    return false;
}

void OBD2Handler::calcConsumption(uint8_t pid, float value) {
    //using namespace Config;

    if (pid == VEHICLE_SPEED) Config::lastSpeed = value;
    if (pid == ENGINE_FUEL_RATE) Config::lastFuelRate = value;

    if (Config::lastSpeed < 1.0 || Config::lastFuelRate < 0.1) return;

    float Config::consumption = (Config::lastFuelRate / Config::lastSpeed) * 100.0;
    Config::consumptionSum += Config::consumption;
    Config::consumptionCount++;

    if (Config::consumptionCount >= 10) {
        float avg = Config::consumptionSum / Config::consumptionCount;
        Utils::sendOBDValue(0xFE, "FUEL_CONS", avg, "L100");

        Config::consumptionSum = 0;
        Config::consumptionCount = 0;
    }
}

void OBD2Handler::requestAndSendPID(uint8_t pid) {
    if (!sendRequest(Config::ObdRequestedPids, pid)) {
        Utils::sendOBDError(pid, getPIDName(pid));
        return;
    }

    uint8_t responseData[8];
    uint8_t responseLen = 0;

    if (receiveResponse(Config::ObdRequestedPids, pid, responseData, responseLen)) {

if Config::sendRawDataOnly
        Utils::sendRawOBDData(pid, responseData, responseLen);
else
        PIDResult result = convertPID(pid, responseData, responseLen);

        if (pid == VEHICLE_SPEED || pid == ENGINE_FUEL_RATE) {
            calcConsumption(pid, result.value);
        }

        if (pid != VEHICLE_SPEED && pid != ENGINE_FUEL_RATE) {
            Utils::sendOBDValue(pid, getPIDName(pid), result.value, result.unit);
        }
endif

    } else {
        Utils::sendOBDError(pid, getPIDName(pid));
    }
}