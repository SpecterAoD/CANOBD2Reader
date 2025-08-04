#pragma once
#include <Arduino.h>
#include <driver/twai.h>
#include "Logger.h"
#include "config.h"

class CANHandler {
public:
    /// @brief Initialisiert den TWAI/CAN-Bus
    static bool init();

    /// @brief Behandelt eingehende CAN-Nachricht
    static void handleMessage(twai_message_t& message);

    /// @brief Prüft und druckt den aktuellen Status (optional)
    static void printStatus();

private:
    /// @brief Aktiviert Alerts und startet den Bus
    static bool configureAlerts();
};