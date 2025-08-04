#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <deque>
#include "Config.h"

class WebConsoleHandler {
public:
    static void begin();
    static void handle();
    static void log(const String& msg);

    /// @brief Gibt an, ob Start gedrückt wurde
    static bool isStarted() { return startRequested || !Config::RequireWebStart; }

private:
    static inline WebServer server{Config::WebConsolePort};
    static inline std::deque<String> logBuffer{};
    static inline bool startRequested = false;

    static void handleRoot();
    static void handleLog();
    static void handleStart();
};