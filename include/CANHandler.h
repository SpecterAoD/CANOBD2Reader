#pragma once
#include <Arduino.h>
#include <driver/twai.h>
#include "Logger.h"
#include "Config.h"

class CANHandler {
public:
    /// @brief Initialisiert den TWAI/CAN-Bus
    static bool init();

    /// @brief Empfängt ausstehende CAN-Nachrichten und verarbeitet sie
    static void processIncoming();

    /// @brief Behandelt eingehende CAN-Nachricht
    /// Neben dem binaeren Rohframe wird eine lesbare CAN-Zusammenfassung fuer
    /// die Display-CAN-Seite erzeugt.
    static void handleMessage(twai_message_t& message);

    /// @brief Prüft und druckt den aktuellen Status (optional)
    static void printStatus();

private:
    /// @brief Aktiviert Alerts und startet den Bus
    static bool configureAlerts();
};
