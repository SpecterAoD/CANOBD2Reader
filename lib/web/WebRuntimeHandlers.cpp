#include "WebRuntimeHandlers.h"

#if defined(ARDUINO)
  #include <Esp.h>
  #include <Update.h>
#endif

#if defined(ARDUINO)
  #include "DiagnosticLog.h"
  #include "config/ProjectConfig.h"
  #include "config/SecurityConfig.h"
#endif
#include "RuntimeSimulation.h"

namespace {

bool asciiEqualsIgnoreCase(char a, char b) {
    if (a >= 'A' && a <= 'Z') a = static_cast<char>(a - 'A' + 'a');
    if (b >= 'A' && b <= 'Z') b = static_cast<char>(b - 'A' + 'a');
    return a == b;
}

bool containsIgnoreCase(const char* haystack, const char* needle) {
    if (haystack == nullptr || needle == nullptr || needle[0] == '\0') return false;

    for (const char* cursor = haystack; *cursor != '\0'; ++cursor) {
        const char* h = cursor;
        const char* n = needle;
        while (*h != '\0' && *n != '\0' && asciiEqualsIgnoreCase(*h, *n)) {
            ++h;
            ++n;
        }
        if (*n == '\0') return true;
    }

    return false;
}

#if defined(ARDUINO)
bool otaSessionActive = false;
bool otaSessionRejected = false;
#endif

} // namespace

namespace WebRuntimeHandlers {

String jsonEscape(const String& value) {
    String escaped;
    escaped.reserve(value.length() + 8);
    for (uint16_t i = 0; i < value.length(); ++i) {
        const char c = value[i];
        if (c == '"' || c == '\\') {
            escaped += '\\';
            escaped += c;
        } else if (c == '\n') {
            escaped += "\\n";
        } else if (c == '\r') {
            escaped += "\\r";
        } else {
            escaped += c;
        }
    }
    return escaped;
}

String simulationJson() {
    String json;
    json.reserve(160);
    json += "{";
    json += "\"simulation\":";
    json += Simulation::RuntimeSimulation::enabled() ? "true" : "false";
    json += ",\"scenario\":\"";
    json += jsonEscape(Simulation::RuntimeSimulation::scenarioName());
    json += "\"}";
    return json;
}

bool firmwareFilenameMatchesTarget(const char* filename, const char* expectedTarget) {
    if (expectedTarget == nullptr || expectedTarget[0] == '\0') return false;
    if (containsIgnoreCase(expectedTarget, "unknown")) return true;
    if (filename == nullptr || filename[0] == '\0') return false;

    const bool isSenderTarget = containsIgnoreCase(expectedTarget, "sender");
    const bool isDisplayTarget = containsIgnoreCase(expectedTarget, "display");

    if (isSenderTarget) {
        return containsIgnoreCase(filename, "sender") && !containsIgnoreCase(filename, "display");
    }
    if (isDisplayTarget) {
        return containsIgnoreCase(filename, "display") && !containsIgnoreCase(filename, "sender");
    }

    return containsIgnoreCase(filename, expectedTarget);
}

#if defined(ARDUINO)
String updateErrorText(const char* prefix) {
    return String(prefix == nullptr ? "Update error" : prefix) + ", error=" + String(Update.getError());
}

bool beginWebOtaUpload(const String& filename, String& status, LogCallback logCallback) {
    otaSessionActive = false;
    otaSessionRejected = false;

    status = "Upload gestartet: " + filename;
    if (logCallback != nullptr) logCallback("[WebOTA] " + status);

    if (SecurityConfig::RequireOtaTargetInFilename &&
        !firmwareFilenameMatchesTarget(filename.c_str(), ProjectConfig::TargetName)) {
        otaSessionRejected = true;
        status = String("OTA abgelehnt: Datei passt nicht zu Target ") + ProjectConfig::TargetName +
                 " (erwartet z.B. " + ProjectConfig::TargetName + ".bin)";
        if (logCallback != nullptr) logCallback("[WebOTA] " + status);
        return false;
    }

    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        otaSessionRejected = true;
        status = updateErrorText("Update.begin fehlgeschlagen");
        if (logCallback != nullptr) logCallback("[WebOTA] " + status);
        return false;
    }

    otaSessionActive = true;
    return true;
}

bool writeWebOtaChunk(uint8_t* data, size_t size, String& status, LogCallback logCallback) {
    if (data == nullptr || size == 0) return true;
    if (otaSessionRejected) return false;

    if (!otaSessionActive) {
        status = "Web-OTA Schreibfehler: kein aktiver Upload";
        if (logCallback != nullptr) logCallback("[WebOTA] " + status);
        return false;
    }

    if (Update.write(data, size) != size) {
        status = updateErrorText("Web-OTA Schreibfehler");
        if (logCallback != nullptr) logCallback("[WebOTA] " + status);
        return false;
    }
    return true;
}

bool finishWebOtaUpload(size_t totalSize, String& status, LogCallback logCallback) {
    if (otaSessionRejected) {
        otaSessionRejected = false;
        otaSessionActive = false;
        if (logCallback != nullptr) logCallback("[WebOTA] Upload verworfen");
        return false;
    }

    if (!otaSessionActive) {
        status = "Update.end fehlgeschlagen: kein aktiver Upload";
        if (logCallback != nullptr) logCallback("[WebOTA] " + status);
        return false;
    }

    if (Update.end(true)) {
        otaSessionActive = false;
        status = "Web-OTA abgeschlossen: " + String(totalSize) + " Bytes";
        if (logCallback != nullptr) logCallback("[WebOTA] " + status);
        return true;
    }

    otaSessionActive = false;
    status = updateErrorText("Update.end fehlgeschlagen");
    if (logCallback != nullptr) logCallback("[WebOTA] " + status);
    return false;
}

void abortWebOtaUpload(String& status, LogCallback logCallback) {
    if (otaSessionActive) {
        Update.end();
    }
    otaSessionActive = false;
    otaSessionRejected = false;
    status = "Web-OTA abgebrochen";
    if (logCallback != nullptr) logCallback("[WebOTA] " + status);
}

void appendFirmwareJson(String& json, const String& otaStatus) {
    json += "\"firmware\":\"" + jsonEscape(ProjectConfig::FirmwareVersion) + "\",";
    json += "\"target\":\"" + jsonEscape(ProjectConfig::TargetName) + "\",";
    json += "\"protocol\":" + String(ProjectConfig::ProtocolVersion) + ",";
    json += "\"buildTime\":\"" + jsonEscape(String(__DATE__) + " " + String(__TIME__)) + "\",";
    json += "\"otaStatus\":\"" + jsonEscape(otaStatus) + "\",";
    json += "\"otaFilenameHint\":\"" + jsonEscape(String(ProjectConfig::TargetName) + ".bin") + "\",";
    json += "\"otaTargetFilenameRequired\":" + String(SecurityConfig::RequireOtaTargetInFilename ? "true" : "false") + ",";
    json += "\"freeSketchSpace\":" + String(ESP.getFreeSketchSpace()) + ",";
    json += "\"sketchSize\":" + String(ESP.getSketchSize()) + ",";
    json += "\"flashSize\":" + String(ESP.getFlashChipSize()) + ",";
}

void appendDiagnosticLogJson(String& json, size_t maxBytes) {
    json += "\"diagnosticLogMounted\":" + String(DiagnosticLog::mounted() ? "true" : "false") + ",";
    json += "\"diagnosticLogSize\":" + String(DiagnosticLog::size()) + ",";
    json += "\"diagnosticLogMaxSize\":" + String(maxBytes) + ",";
}

void sendRestartResponseAndRestart(WebServer& server, uint32_t delayMs, LogCallback logCallback) {
    if (logCallback != nullptr) {
        logCallback("[Web] Neustart angefordert");
    }
    server.send(200, "application/json", "{\"restart\":true}");
    delay(delayMs);
    ESP.restart();
}
#endif

} // namespace WebRuntimeHandlers
