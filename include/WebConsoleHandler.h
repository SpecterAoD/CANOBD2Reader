#pragma once

#include <Arduino.h>
#include <deque>

#include "common_config.h"
#include "config/SenderConfig.h"
#include "WebRuntimeStatus.h"

// Sender web console (status, diagnostics, capability scanner). Shared
// management features come from lib/web/ManagementServer.
class WebConsoleHandler {
public:
    static void begin();
    static void handle();
    static void log(const String& msg);
    static void recordTelemetry(const char* payload);
    static void updateRuntimeStatus(const Runtime::WebRuntimeStatus& status);
    // Firmware updates are refused while this returns true (vehicle moving).
    static void setInstallBlocker(bool (*blocker)());
    static String diagnosticTextReport();

    /// Whether the sender may run (auto start, or released via the web UI).
    static bool isStarted() {
#if CANOBD2_ENABLE_SENDER_WEBCONSOLE
        return startRequested || !SenderConfig::RequireWebStart;
#else
        return true;
#endif
    }

private:
    static inline std::deque<String> logBuffer{};
    static inline bool startRequested = false;
    static inline bool (*installBlocker)() = nullptr;
    static inline Runtime::WebRuntimeStatus runtimeStatus{};

    static void handleRoot();
    static void handleLog();
    static void handleStatus();
    static void handleDiagnosticSnapshot();
    static void handleDiagnosticDownload();
    static void handleStart();
    static void handleCapabilitiesStatus();
    static void handleCapabilitiesExport();
    static void handleCapabilitiesObdStart();
    static void handleCapabilitiesUdsStart();
    static void handleCapabilitiesCanStart();
    static void handleCapabilitiesCanBaseline();
    static void handleCapabilitiesCanActionStart();
    static void handleCapabilitiesCanActionFinish();
    static void handleCapabilitiesStop();
    static void appendLiveWebBuffer(String& report);
    static String diagnosticSnapshotJson();
};
