#include "WebRuntimeHandlers.h"

#include <cstddef>
#include <cstdio>
#include <cstring>

#if defined(ARDUINO)
  #include <Esp.h>
  #include <Update.h>
  #include <mbedtls/sha256.h>
#endif

#if defined(ARDUINO)
  #include "DiagnosticLog.h"
  #include "AuthHelpers.h"
  #include "FirmwareMetadata.h"
  #include "config/ProjectConfig.h"
  #include "config/SecurityConfig.h"
#endif
#include "RuntimeSimulation.h"

namespace {

// Tail buffer for sliding-window metadata checks across OTA chunks.
constexpr size_t kOtaSearchTailSize = 160;
constexpr size_t kOtaCombinedSearchWindowSize = kOtaSearchTailSize * 2;
constexpr size_t kSha256DigestSize = 32;
constexpr size_t kSha256HexStringSize = (kSha256DigestSize * 2) + 1;
constexpr const char* kFirmwareMetadataBegin = "CANOBD2_FW_METADATA_BEGIN";
constexpr const char* kFirmwareVersionMarkerPrefix = "version=V";
constexpr const char* kFirmwareMetadataSchemaPrefix = "schema=";

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

bool isAsciiAlphaNumeric(char value) {
    return (value >= 'a' && value <= 'z') ||
           (value >= 'A' && value <= 'Z') ||
           (value >= '0' && value <= '9');
}

bool endsWithIgnoreCase(const char* value, const char* suffix) {
    if (value == nullptr || suffix == nullptr) return false;

    std::size_t valueLength = 0;
    while (value[valueLength] != '\0') ++valueLength;
    std::size_t suffixLength = 0;
    while (suffix[suffixLength] != '\0') ++suffixLength;
    if (suffixLength == 0 || suffixLength > valueLength) return false;

    const std::size_t start = valueLength - suffixLength;
    for (std::size_t index = 0; index < suffixLength; ++index) {
        if (!asciiEqualsIgnoreCase(value[start + index], suffix[index])) return false;
    }
    return true;
}

bool containsTokenIgnoreCase(const char* haystack, const char* needle) {
    if (haystack == nullptr || needle == nullptr || needle[0] == '\0') return false;

    for (const char* cursor = haystack; *cursor != '\0'; ++cursor) {
        const char* h = cursor;
        const char* n = needle;
        while (*h != '\0' && *n != '\0' && asciiEqualsIgnoreCase(*h, *n)) {
            ++h;
            ++n;
        }
        if (*n != '\0') continue;

        const char previous = cursor == haystack ? '\0' : *(cursor - 1);
        const char next = *h;
        const bool leftBoundary = previous == '\0' || !isAsciiAlphaNumeric(previous);
        const bool rightBoundary = next == '\0' || !isAsciiAlphaNumeric(next);
        if (leftBoundary && rightBoundary) return true;
    }

    return false;
}

bool bufferContainsText(const uint8_t* data, size_t size, const char* text) {
    if (data == nullptr || size == 0 || text == nullptr || text[0] == '\0') return false;

    const size_t textLength = std::strlen(text);
    if (textLength == 0 || textLength > size) return false;

    for (size_t index = 0; index + textLength <= size; ++index) {
        if (std::memcmp(data + index, text, textLength) == 0) return true;
    }
    return false;
}

bool bufferContainsAnyFirmwareVersionMarker(const uint8_t* data, size_t size) {
    return bufferContainsText(data, size, kFirmwareVersionMarkerPrefix);
}

bool bufferContainsFirmwareMetadataMarker(const uint8_t* data, size_t size) {
    return bufferContainsText(data, size, kFirmwareMetadataBegin);
}

bool bufferContainsTargetMetadata(const uint8_t* data, size_t size, const char* expectedTarget) {
    if (expectedTarget == nullptr || expectedTarget[0] == '\0') return false;
    char marker[32];
    std::snprintf(marker, sizeof(marker), "target=%s", expectedTarget);
    return bufferContainsText(data, size, marker);
}

bool bufferContainsProtocolMetadata(const uint8_t* data, size_t size, uint8_t expectedProtocol) {
    char marker[24];
    std::snprintf(marker, sizeof(marker), "protocol=%u", static_cast<unsigned int>(expectedProtocol));
    return bufferContainsText(data, size, marker);
}

bool bufferContainsMetadataSchema(const uint8_t* data, size_t size) {
    return bufferContainsText(data, size, kFirmwareMetadataSchemaPrefix);
}

void appendJsonEscapedField(String& json, const char* key, const String& value) {
    json += '"';
    json += key;
    json += "\":\"";
    json += WebRuntimeHandlers::jsonEscape(value);
    json += "\",";
}

void appendJsonEscapedField(String& json, const char* key, const char* value) {
    appendJsonEscapedField(json, key, String(value == nullptr ? "" : value));
}

void appendJsonBoolField(String& json, const char* key, bool value) {
    json += '"';
    json += key;
    json += "\":";
    json += value ? "true," : "false,";
}

void appendJsonUnsignedField(String& json, const char* key, unsigned long value) {
    char buffer[24];
    std::snprintf(buffer, sizeof(buffer), "%lu", value);
    json += '"';
    json += key;
    json += "\":";
    json += buffer;
    json += ',';
}

#if defined(ARDUINO)
bool otaSessionActive = false;
bool otaSessionRejected = false;
bool otaSessionMetadataSeen = false;
bool otaSessionSchemaSeen = false;
bool otaSessionTargetSeen = false;
bool otaSessionVersionSeen = false;
bool otaSessionProtocolSeen = false;
char otaSessionSha256[kSha256HexStringSize] = {};
uint8_t otaSearchTail[kOtaSearchTailSize] = {};
size_t otaSearchTailLength = 0;
mbedtls_sha256_context otaSha256Context;

// SHA-256 is computed incrementally over each incoming chunk so that no
// second-pass read of the flash image is required. The hash is finalized
// only after Update.end() confirms the full write was accepted, ensuring
// the digest covers exactly the bytes that were written to flash.
void resetOtaValidationState() {
    mbedtls_sha256_free(&otaSha256Context);
    otaSessionMetadataSeen = false;
    otaSessionSchemaSeen = false;
    otaSessionTargetSeen = false;
    otaSessionVersionSeen = false;
    otaSessionProtocolSeen = false;
    otaSessionSha256[0] = '\0';
    otaSearchTailLength = 0;
    std::memset(otaSearchTail, 0, sizeof(otaSearchTail));
    mbedtls_sha256_init(&otaSha256Context);
    mbedtls_sha256_starts_ret(&otaSha256Context, 0);
}

// Firmware metadata markers (target, schema, version, protocol) can straddle
// a chunk boundary because the HTTP upload handler delivers data in variable-
// sized pieces. The sliding tail buffer retains the last kOtaSearchTailSize
// bytes of the previous chunk so that a combined window spanning two chunks
// can be searched, ensuring no marker is missed at the boundary.
void updateOtaValidationState(const uint8_t* data, size_t size) {
    if (data == nullptr || size == 0) return;

    mbedtls_sha256_update_ret(&otaSha256Context, data, size);

    uint8_t combined[kOtaCombinedSearchWindowSize];
    size_t combinedSize = 0;
    if (otaSearchTailLength > 0) {
        std::memcpy(combined, otaSearchTail, otaSearchTailLength);
        combinedSize = otaSearchTailLength;
    }
    const size_t copySize = size > sizeof(combined) - combinedSize ? sizeof(combined) - combinedSize : size;
    std::memcpy(combined + combinedSize, data, copySize);
    combinedSize += copySize;

    // Each validation flag is set once and never cleared within a session.
    // Checking both the raw chunk and the combined window ensures detection
    // even when a marker begins in one chunk and ends in the next.
    if (!otaSessionTargetSeen) {
        otaSessionTargetSeen =
            bufferContainsTargetMetadata(data, size, ProjectConfig::TargetName) ||
            bufferContainsTargetMetadata(combined, combinedSize, ProjectConfig::TargetName) ||
            bufferContainsText(data, size, ProjectConfig::TargetName) ||
            bufferContainsText(combined, combinedSize, ProjectConfig::TargetName);
    }
    if (!otaSessionMetadataSeen) {
        otaSessionMetadataSeen =
            bufferContainsFirmwareMetadataMarker(data, size) ||
            bufferContainsFirmwareMetadataMarker(combined, combinedSize);
    }
    if (!otaSessionSchemaSeen) {
        otaSessionSchemaSeen =
            bufferContainsMetadataSchema(data, size) ||
            bufferContainsMetadataSchema(combined, combinedSize);
    }
    if (!otaSessionVersionSeen) {
        otaSessionVersionSeen =
            (bufferContainsFirmwareMetadataMarker(data, size) &&
             bufferContainsAnyFirmwareVersionMarker(data, size)) ||
            (bufferContainsFirmwareMetadataMarker(combined, combinedSize) &&
             bufferContainsAnyFirmwareVersionMarker(combined, combinedSize)) ||
            bufferContainsText(data, size, ProjectConfig::FirmwareVersion) ||
            bufferContainsText(combined, combinedSize, ProjectConfig::FirmwareVersion);
    }
    // Protocol version must match exactly; accepting a mismatched protocol would
    // allow incompatible display/sender pairs to silently exchange garbled telemetry.
    if (!otaSessionProtocolSeen) {
        otaSessionProtocolSeen =
            bufferContainsProtocolMetadata(data, size, ProjectConfig::ProtocolVersion) ||
            bufferContainsProtocolMetadata(combined, combinedSize, ProjectConfig::ProtocolVersion);
    }

    if (size >= sizeof(otaSearchTail)) {
        std::memcpy(otaSearchTail, data + size - sizeof(otaSearchTail), sizeof(otaSearchTail));
        otaSearchTailLength = sizeof(otaSearchTail);
    } else {
        const size_t keepPrefix = otaSearchTailLength + size > sizeof(otaSearchTail)
                                      ? otaSearchTailLength + size - sizeof(otaSearchTail)
                                      : 0;
        if (keepPrefix > 0 && keepPrefix < otaSearchTailLength) {
            std::memmove(otaSearchTail, otaSearchTail + keepPrefix, otaSearchTailLength - keepPrefix);
            otaSearchTailLength -= keepPrefix;
        } else if (keepPrefix >= otaSearchTailLength) {
            otaSearchTailLength = 0;
        }
        std::memcpy(otaSearchTail + otaSearchTailLength, data, size);
        otaSearchTailLength += size;
    }
}

void finalizeOtaSha256() {
    uint8_t digest[kSha256DigestSize];
    mbedtls_sha256_finish_ret(&otaSha256Context, digest);
    for (size_t i = 0; i < sizeof(digest); ++i) {
        std::snprintf(otaSessionSha256 + (i * 2), 3, "%02x", digest[i]);
    }
    otaSessionSha256[kSha256HexStringSize - 1] = '\0';
    mbedtls_sha256_free(&otaSha256Context);
}
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
    if (!endsWithIgnoreCase(filename, ".bin")) return false;
    if (containsIgnoreCase(filename, "/") || containsIgnoreCase(filename, "\\")) return false;

    const bool isSenderTarget = containsTokenIgnoreCase(expectedTarget, "sender");
    const bool isDisplayTarget = containsTokenIgnoreCase(expectedTarget, "display");

    if (isSenderTarget) {
        return containsTokenIgnoreCase(filename, "sender") && !containsTokenIgnoreCase(filename, "display");
    }
    if (isDisplayTarget) {
        return containsTokenIgnoreCase(filename, "display") && !containsTokenIgnoreCase(filename, "sender");
    }

    return containsTokenIgnoreCase(filename, expectedTarget);
}

bool firmwareBufferContainsText(const uint8_t* data, size_t size, const char* text) {
    return bufferContainsText(data, size, text);
}

bool firmwareBufferContainsTargetMarker(const uint8_t* data, size_t size, const char* expectedTarget) {
    if (expectedTarget == nullptr || expectedTarget[0] == '\0') return false;
    return bufferContainsTargetMetadata(data, size, expectedTarget) ||
           firmwareBufferContainsText(data, size, expectedTarget);
}

bool firmwareBufferContainsVersionMarker(const uint8_t* data, size_t size, const char* expectedVersion) {
    if (expectedVersion == nullptr || expectedVersion[0] == '\0') return false;
    return (firmwareBufferContainsText(data, size, kFirmwareMetadataBegin) &&
            bufferContainsAnyFirmwareVersionMarker(data, size)) ||
           firmwareBufferContainsText(data, size, expectedVersion);
}

bool firmwareBufferContainsProtocolMarker(const uint8_t* data, size_t size, uint8_t expectedProtocol) {
    return bufferContainsFirmwareMetadataMarker(data, size) &&
           bufferContainsMetadataSchema(data, size) &&
           bufferContainsProtocolMetadata(data, size, expectedProtocol);
}

#if defined(ARDUINO)
String updateErrorText(const char* prefix) {
    return String(prefix == nullptr ? "Update error" : prefix) + ", error=" + String(Update.getError());
}

bool beginWebOtaUpload(const String& filename, String& status, LogCallback logCallback) {
    otaSessionActive = false;
    otaSessionRejected = false;
    resetOtaValidationState();

    status = "Upload gestartet: " + filename;
    if (logCallback != nullptr) logCallback("[WebOTA] " + status);

    const String securityWarning = WebSecurity::targetSecurityWarning(ProjectConfig::TargetName, false);
    if (SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets && securityWarning.length() > 0) {
        otaSessionRejected = true;
        status = "OTA abgelehnt: unsichere Platzhalter-Secrets aktiv (" + securityWarning + ")";
        if (logCallback != nullptr) logCallback("[WebOTA] " + status);
        return false;
    }

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

    updateOtaValidationState(data, size);

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

    finalizeOtaSha256();

    if (!otaSessionMetadataSeen || !otaSessionSchemaSeen || !otaSessionTargetSeen || !otaSessionVersionSeen || !otaSessionProtocolSeen) {
        Update.end();
        otaSessionActive = false;
        status = String("OTA abgelehnt: Firmware-Metadaten ungueltig (target=") +
                 (otaSessionTargetSeen ? "ok" : "fehlt") + ", version=" +
                 (otaSessionVersionSeen ? "ok" : "fehlt") + ", protocol=" +
                 (otaSessionProtocolSeen ? "ok" : "fehlt") + ", metadata=" +
                 (otaSessionMetadataSeen ? "ok" : "fehlt") + ", schema=" +
                 (otaSessionSchemaSeen ? "ok" : "fehlt") + ")";
        if (logCallback != nullptr) {
            logCallback("[WebOTA] " + status);
            logCallback(String("[WebOTA] sha256=") + otaSessionSha256);
        }
        return false;
    }

    if (Update.end(true)) {
        otaSessionActive = false;
        status = "Web-OTA abgeschlossen: " + String(totalSize) + " Bytes";
        if (logCallback != nullptr) {
            logCallback("[WebOTA] " + status);
            logCallback(String("[WebOTA] sha256=") + otaSessionSha256);
        }
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
    otaSessionSha256[0] = '\0';
    status = "Web-OTA abgebrochen";
    if (logCallback != nullptr) logCallback("[WebOTA] " + status);
}

void appendFirmwareJson(String& json, const String& otaStatus) {
    const String securityWarning = WebSecurity::targetSecurityWarning(ProjectConfig::TargetName);
    char buildTime[32];
    std::snprintf(buildTime, sizeof(buildTime), "%s %s", __DATE__, __TIME__);
    char otaFilenameHint[48];
    std::snprintf(otaFilenameHint, sizeof(otaFilenameHint), "%s.bin", ProjectConfig::TargetName);

    appendJsonEscapedField(json, "firmware", ProjectConfig::FirmwareVersion);
    appendJsonEscapedField(json, "target", ProjectConfig::TargetName);
    appendJsonUnsignedField(json, "protocol", ProjectConfig::ProtocolVersion);
    appendJsonEscapedField(json, "buildTime", buildTime);
    appendJsonEscapedField(json, "otaStatus", otaStatus);
    appendJsonEscapedField(json, "otaFilenameHint", otaFilenameHint);
    appendJsonEscapedField(json, "otaMetadata", FirmwareMetadata::text());
    appendJsonBoolField(json, "otaTargetFilenameRequired", SecurityConfig::RequireOtaTargetInFilename);
    appendJsonBoolField(json, "otaMetadataRequired", true);
    appendJsonBoolField(json, "securityReady", securityWarning.length() == 0);
    appendJsonEscapedField(json, "securityWarning", securityWarning);
    appendJsonUnsignedField(json, "freeSketchSpace", ESP.getFreeSketchSpace());
    appendJsonUnsignedField(json, "sketchSize", ESP.getSketchSize());
    appendJsonUnsignedField(json, "flashSize", ESP.getFlashChipSize());
#if defined(ARDUINO)
    appendJsonEscapedField(json, "otaLastSha256", otaSessionSha256);
#endif
}

void appendDiagnosticLogJson(String& json, size_t maxBytes) {
    appendJsonBoolField(json, "diagnosticLogMounted", DiagnosticLog::mounted());
    appendJsonUnsignedField(json, "diagnosticLogSize", DiagnosticLog::size());
    appendJsonUnsignedField(json, "diagnosticLogMaxSize", maxBytes);
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
