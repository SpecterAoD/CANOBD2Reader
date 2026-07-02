#pragma once

#include <cstdint>

#if defined(ARDUINO)
  #include <Arduino.h>
  #include <WebServer.h>
#else
  #include <string>
  using String = std::string;
#endif

namespace WebRuntimeHandlers {

#if defined(ARDUINO)
using LogCallback = void (*)(const String& message);
#endif

String jsonEscape(const String& value);
String simulationJson();
bool firmwareFilenameMatchesTarget(const char* filename, const char* expectedTarget);
bool firmwareBufferContainsText(const uint8_t* data, size_t size, const char* text);
bool firmwareBufferContainsTargetMarker(const uint8_t* data, size_t size, const char* expectedTarget);
bool firmwareBufferContainsVersionMarker(const uint8_t* data, size_t size, const char* expectedVersion);

#if defined(ARDUINO)
String updateErrorText(const char* prefix);
bool beginWebOtaUpload(const String& filename, String& status, LogCallback logCallback = nullptr);
bool writeWebOtaChunk(uint8_t* data, size_t size, String& status, LogCallback logCallback = nullptr);
bool finishWebOtaUpload(size_t totalSize, String& status, LogCallback logCallback = nullptr);
void abortWebOtaUpload(String& status, LogCallback logCallback = nullptr);
void appendFirmwareJson(String& json, const String& otaStatus);
void appendDiagnosticLogJson(String& json, size_t maxBytes);
void sendRestartResponseAndRestart(WebServer& server, uint32_t delayMs, LogCallback logCallback = nullptr);
#endif

} // namespace WebRuntimeHandlers
