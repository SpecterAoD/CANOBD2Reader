#pragma once

#include <Arduino.h>
#include <driver/twai.h>

#include "Config.h"
#include "Logger.h"

class CANHandler {
public:
    /// Initializes the ESP32 TWAI/CAN driver.
    static bool init();

    /// Reads all pending CAN frames and forwards them to handleMessage().
    static void processIncoming();

    /// Converts one CAN frame into telemetry/debug output.
    static void handleMessage(twai_message_t& message);

    /// Prints the current TWAI state for diagnostics.
    static void printStatus();

    /// Stops and uninstalls the TWAI driver if it was started.
    static void shutdown();

private:
    static bool configureAlerts();
    static bool driverInstalled;
    static bool driverStarted;
};
