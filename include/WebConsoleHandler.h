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
    static void handleCapabilitiesStatus();
    static void handleCapabilitiesExport();
    static void handleCapabilitiesObdStart();
    static void handleCapabilitiesUdsStart();
    static void handleCapabilitiesCanStart();
    static void handleCapabilitiesCanBaseline();
    static void handleCapabilitiesCanActionStart();
    static void handleCapabilitiesCanActionFinish();
    static void handleCapabilitiesStop();
    static void handleGithubUpdatePage();
    static void handleWifiStatus();
    static void handleWifiConfigure();
    static void handleWifiConnect();
    static void handleWifiForget();
    static void handleGithubUpdateStatus();
    static void handleGithubUpdateVersions();
    static void handleGithubUpdateCheck();
    static void handleGithubUpdateInstall();
    static void handleGithubUpdateChannel();
    static void handleUpdatePage();
    static void handleUpdateFinished();
    static void handleUpdateUpload();
    static String simulationJson();
    static String jsonEscape(const String& value);
    static void appendLiveWebBuffer(String& report);
    static String diagnosticSnapshotJson();
    static String diagnosticTextReport();
};
