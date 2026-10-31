#pragma once

// Management access point + web server shared by sender and display.
//
// Common endpoints (all authenticated):
//   POST /update                    signed firmware upload (multipart, field "firmware")
//   GET  /github-update             GitHub update page
//   GET  /api/update/status|versions, POST /api/update/check|install|channel
//   GET  /api/wifi/status,          POST /api/wifi/configure|connect|forget
//   GET  /log/file|/log/download,   POST /api/log/clear
//   GET  /api/simulation,           POST /api/simulation/on|off|toggle|scenario
//   POST /api/restart
// Device specific pages (/, /status, ...) are registered by the target via
// server() after begin().
//
// Nothing is started when the device has no provisioned credentials
// (see lib/security/DeviceSecrets.h).

#if defined(ARDUINO)

#include <Arduino.h>
#include <WebServer.h>

#include "FirmwareUpdateManager.h"

namespace Management {

using LogFunction = void (*)(const String& message);
using ReportFunction = String (*)();

struct Config {
    const char* deviceName;           // "Sender" or "Display", used in page titles
    const char* apSsid;
    const char* apPassword;
    LogFunction log;                  // required
    ReportFunction diagnosticReport;  // full text report for /log/file
    FirmwareUpdate::FirmwareUpdateManager::InstallBlocker installBlocker; // may be nullptr
};

// Starts AP, station manager, update manager and the common routes.
// Returns false (and starts nothing) when credentials are missing.
bool begin(const Config& config);
void handle();
bool running();
WebServer& server();

bool otaInProgress();
const String& otaStatus();

// Appends common fields (firmware, OTA, log, security, AP) to a JSON object
// body; every field ends with ',' so the caller can continue the object.
void appendCommonStatusJson(String& json);

String jsonEscape(const String& value);
// Sends the response and restarts after a short delay.
void sendRestartAndRestart();

} // namespace Management

#endif
