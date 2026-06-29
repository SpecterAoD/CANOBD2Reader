#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <deque>
#include "common_config.h"
#include "Config.h"

struct WebConsoleRuntimeStatus {
    bool canActive = false;
    bool obdActive = false;
    bool pidSupportReady = false;
    bool simulationActive = false;
    float batteryVoltage = 0.0f;
    uint32_t uptimeMs = 0;
    uint32_t telemetrySequence = 0;
    uint32_t lastCanAgeMs = 0;
    String canState = "UNKNOWN";
    String lastDtc = "--";
    String lastTelemetry = "--";
    String lastError = "";
};

class WebConsoleHandler {
public:
    static void begin();
    static void handle();
    static void log(const String& msg);
    static void recordTelemetry(const char* payload);
    static void updateRuntimeStatus(const WebConsoleRuntimeStatus& status);

    /// @brief Gibt an, ob der Benutzer den Start auf der WebConsole freigegeben hat.
    static bool isStarted() {
#if CANOBD2_ENABLE_SENDER_WEBCONSOLE
        return startRequested || !Config::Network::RequireWebStart;
#else
        return true;
#endif
    }

private:
    static inline WebServer server{Config::Network::WebServerPort};
    static inline std::deque<String> logBuffer{};
    static inline bool startRequested = false;
    static inline bool updateInProgress = false;
    static inline WebConsoleRuntimeStatus runtimeStatus{};

    static void handleRoot();
    static void handleLog();
    static void handleStatus();
    static void handleStart();
    static void handleRestart();
    static void handleUpdatePage();
    static void handleUpdateFinished();
    static void handleUpdateUpload();
    static String jsonEscape(const String& value);
};
