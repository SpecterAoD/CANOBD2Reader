#pragma once

#include <cstddef>
#include <cstdint>

namespace LoggingConfig {

constexpr bool SerialEnabled = true;
constexpr bool GeneralDebugEnabled = true;
constexpr bool TwaiDebugEnabled = false;
constexpr bool PowerDebugEnabled = false;
constexpr bool CanDebugEnabled = false;
constexpr bool Obd2DebugEnabled = false;
constexpr bool TraceSenderTelemetry = false;
constexpr bool TraceDisplayTelemetry = false;
constexpr uint32_t TraceSummaryIntervalMs = 1000;

constexpr bool EnablePersistentDiagnosticLog = true;
constexpr bool PersistTelemetryPayloadsToDiagnosticLog = false;
constexpr std::size_t DiagnosticLogMaxBytes = 64U * 1024U;

constexpr const char* SenderDiagnosticLogPath = "/sender_diagnostic.log";
constexpr const char* SenderDiagnosticLogArchivePath = "/sender_diagnostic.old";
constexpr const char* DisplayDiagnosticLogPath = "/display_diagnostic.log";
constexpr const char* DisplayDiagnosticLogArchivePath = "/display_diagnostic.old";

constexpr uint32_t TwaiStatusLogIntervalMs = 5000;

}
