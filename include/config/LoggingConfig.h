#pragma once

#include <cstddef>
#include <cstdint>

namespace LoggingConfig {

constexpr bool SerialEnabled = true;
constexpr bool GeneralDebugEnabled = true;
constexpr bool TwaiDebugEnabled = true;
constexpr bool PowerDebugEnabled = true;
constexpr bool CanDebugEnabled = true;
constexpr bool Obd2DebugEnabled = true;
constexpr bool TraceSenderTelemetry = true;
constexpr bool TraceDisplayTelemetry = true;
constexpr uint32_t TraceSummaryIntervalMs = 1000;

constexpr bool EnablePersistentDiagnosticLog = true;
constexpr bool PersistTelemetryPayloadsToDiagnosticLog = true;
constexpr std::size_t DiagnosticLogMaxBytes = 64U * 1024U;

constexpr const char* SenderDiagnosticLogPath = "/sender_diagnostic.log";
constexpr const char* SenderDiagnosticLogArchivePath = "/sender_diagnostic.old";
constexpr const char* DisplayDiagnosticLogPath = "/display_diagnostic.log";
constexpr const char* DisplayDiagnosticLogArchivePath = "/display_diagnostic.old";

constexpr uint32_t TwaiStatusLogIntervalMs = 5000;

}
