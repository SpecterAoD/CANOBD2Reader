#pragma once

#include <cstddef>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

namespace DiagnosticLog {

/// Mounts the persistent diagnostic log storage.
bool begin();

/// Appends one already formatted line. The function never logs recursively.
void append(const char* line);

/// Appends a timestamped printf-style line.
void appendf(const char* format, ...);

/// Removes current and archived diagnostic log files.
bool clear();

/// Returns the current diagnostic log file size in bytes.
std::size_t size();

/// Returns true after the persistent storage was mounted successfully.
bool mounted();

/// Small pure helper used by native tests and the Arduino implementation.
bool shouldRotate(std::size_t currentSize, std::size_t incomingBytes, std::size_t maxBytes);

#if defined(ARDUINO)
/// Reads archive and current log as plain text for WebConsole downloads.
String readAll();
#endif

}
