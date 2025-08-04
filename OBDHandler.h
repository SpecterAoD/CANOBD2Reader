#pragma once
#include <Arduino.h>
#include <driver/twai.h>
#include "Logger.h"
#include "config.h"

class OBD2Handler {
public:
    /// @brief Sendet eine OBD2-Anfrage an ein bestimmtes PID
    static bool sendRequest(uint8_t mode, uint8_t pid);

    /// @brief Empfängt eine Antwort auf ein OBD2-PID
    static bool receiveResponse(uint8_t mode, uint8_t pid, uint8_t* outData, uint8_t& outLen);

    /// @brief Fordert PID an und sendet Ergebnis via Utils
    static void requestAndSendPID(uint8_t pid);

    /// @brief Berechnet und sendet Verbrauch aus Speed + FuelRate
    static void calcConsumption(uint8_t pid, float value);
};