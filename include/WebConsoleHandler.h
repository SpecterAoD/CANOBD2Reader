#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <deque>
#include "common_config.h"
#include "config/SenderConfig.h"
#include "config/NetworkConfig.h"
#include "WebRuntimeStatus.h"

class WebConsoleHandler {
public:
    static void begin();
    static void handle();
    static void log(const String& msg);
    static void recordTelemetry(const char* payload);
    static void updateRuntimeStatus(const Runtime::WebRuntimeStatus& status);

    /// @brief Gibt an, ob der Benutzer den Start auf der WebConsole freigegeben hat.
    static bool isStarted() {
#if CANOBD2_ENABLE_SENDER_WEBCONSOLE
        return startRequested || !SenderConfig::RequireWebStart;
#else
        return true;
#endif
    }

private:
    static inline WebServer server{NetworkConfig::WebServerPort};
    static inline std::deque<String> logBuffer{};
    static inline bool startRequested = false;
    static inline bool updateInProgress = false;
    static inline Runtime::WebRuntimeStatus runtimeStatus{};

    static void handleRoot();
    static void handleLog();
    static void handlePersistentLog();
    static void handleDownloadLog();
    static void handleClearLog();
    static void handleStatus();
    static void handleDiagnosticSnapshot();
    static void handleDiagnosticDownload();
    static void handleStart();
    static void handleRestart();
    static void handleApiRestart();
    static void handleSimulationStatus();
    static void handleSimulationOn();
    static void handleSimulationOff();
    static void handleSimulationToggle();
    static void handleSimulationScenario();
    static void handleUpdatePage();
    static void handleUpdateFinished();
    static void handleUpdateUpload();
    static String simulationJson();
    static String jsonEscape(const String& value);
};
