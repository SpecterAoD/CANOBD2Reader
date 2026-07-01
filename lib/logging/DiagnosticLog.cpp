#include "DiagnosticLog.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#if defined(ARDUINO)
#include <FS.h>
#include <SPIFFS.h>
#include "LoggingConfig.h"
#include "ProjectConfig.h"
#endif

namespace DiagnosticLog {
namespace {
#if defined(ARDUINO)
bool mountedState = false;

bool isDisplayTarget() {
    return strcmp(ProjectConfig::TargetName, "display") == 0;
}

const char* currentPath() {
    return isDisplayTarget() ? LoggingConfig::DisplayDiagnosticLogPath
                             : LoggingConfig::SenderDiagnosticLogPath;
}

const char* archivePath() {
    return isDisplayTarget() ? LoggingConfig::DisplayDiagnosticLogArchivePath
                             : LoggingConfig::SenderDiagnosticLogArchivePath;
}

String timestamped(const char* line) {
    String out;
    out.reserve(strlen(line) + 18);
    out += String(millis() / 1000);
    out += "s ";
    out += line;
    if (!out.endsWith("\n")) out += '\n';
    return out;
}

void rotateIfNeeded(std::size_t incomingBytes) {
    File current = SPIFFS.open(currentPath(), FILE_READ);
    const std::size_t currentSize = current ? current.size() : 0;
    if (current) current.close();

    if (!shouldRotate(currentSize, incomingBytes, LoggingConfig::DiagnosticLogMaxBytes)) return;

    SPIFFS.remove(archivePath());
    SPIFFS.rename(currentPath(), archivePath());
}

void appendRaw(const String& line) {
    if (!LoggingConfig::EnablePersistentDiagnosticLog || !mountedState) return;

    rotateIfNeeded(line.length());
    File file = SPIFFS.open(currentPath(), FILE_APPEND);
    if (!file) return;
    file.print(line);
    file.close();
}

String readFile(const char* path) {
    File file = SPIFFS.open(path, FILE_READ);
    if (!file) return String();

    String content;
    content.reserve(file.size() + 32);
    while (file.available()) {
        content += static_cast<char>(file.read());
    }
    file.close();
    return content;
}
#endif
}

bool shouldRotate(std::size_t currentSize, std::size_t incomingBytes, std::size_t maxBytes) {
    return maxBytes > 0 && currentSize + incomingBytes > maxBytes;
}

bool begin() {
#if defined(ARDUINO)
    if (!LoggingConfig::EnablePersistentDiagnosticLog) return false;
    if (mountedState) return true;

    mountedState = SPIFFS.begin(true);
    if (mountedState) {
        char line[96];
        snprintf(line, sizeof(line), "[DIAG] Persistent diagnostic log mounted target=%s",
                 ProjectConfig::TargetName);
        appendRaw(timestamped(line));
    }
    return mountedState;
#else
    return false;
#endif
}

void append(const char* line) {
#if defined(ARDUINO)
    if (line == nullptr || line[0] == '\0') return;
    appendRaw(String(line) + (line[strlen(line) - 1] == '\n' ? "" : "\n"));
#else
    (void)line;
#endif
}

void appendf(const char* format, ...) {
    if (format == nullptr) return;

    char buffer[192];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

#if defined(ARDUINO)
    appendRaw(timestamped(buffer));
#else
    (void)buffer;
#endif
}

bool clear() {
#if defined(ARDUINO)
    if (!mountedState) return false;
    SPIFFS.remove(currentPath());
    SPIFFS.remove(archivePath());
    return true;
#else
    return true;
#endif
}

std::size_t size() {
#if defined(ARDUINO)
    if (!mountedState) return 0;
    File file = SPIFFS.open(currentPath(), FILE_READ);
    const std::size_t bytes = file ? file.size() : 0;
    if (file) file.close();
    return bytes;
#else
    return 0;
#endif
}

bool mounted() {
#if defined(ARDUINO)
    return mountedState;
#else
    return false;
#endif
}

#if defined(ARDUINO)
String readAll() {
    if (!mountedState) return "Diagnostic log storage is not mounted.\n";

    String content;
    const String archived = readFile(archivePath());
    if (archived.length() > 0) {
        content += "----- archived log -----\n";
        content += archived;
        if (!content.endsWith("\n")) content += '\n';
    }

    const String current = readFile(currentPath());
    if (current.length() > 0) {
        content += "----- current log -----\n";
        content += current;
    }

    if (content.length() == 0) content = "Diagnostic log is empty.\n";
    return content;
}
#endif

}
