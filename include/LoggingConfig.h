#pragma once

#include <cstddef>
#include <cstdint>

namespace LoggingConfig {

constexpr bool EnablePersistentDiagnosticLog = true;
constexpr bool PersistTelemetryPayloadsToDiagnosticLog = true;
constexpr std::size_t DiagnosticLogMaxBytes = 64U * 1024U;

constexpr const char* SenderDiagnosticLogPath = "/sender_diagnostic.log";
constexpr const char* SenderDiagnosticLogArchivePath = "/sender_diagnostic.old";
constexpr const char* DisplayDiagnosticLogPath = "/display_diagnostic.log";
constexpr const char* DisplayDiagnosticLogArchivePath = "/display_diagnostic.old";

constexpr uint32_t TwaiStatusLogIntervalMs = 5000;

}
