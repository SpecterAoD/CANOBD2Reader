#pragma once

#include <cstddef>
#include <cstdint>

namespace LoggingConfig {

constexpr bool SerialEnabled = true;
// Serial debug output per CAN frame / OBD request costs loop time; enable only for bench debugging.
constexpr bool GeneralDebugEnabled = false;
constexpr bool TwaiDebugEnabled = false;
constexpr bool PowerDebugEnabled = false;
constexpr bool CanDebugEnabled = false;
constexpr bool Obd2DebugEnabled = false;
constexpr bool TraceSenderTelemetry = false;
constexpr uint32_t TraceSummaryIntervalMs = 1000;

constexpr bool EnablePersistentDiagnosticLog = true;
constexpr bool PersistTelemetryPayloadsToDiagnosticLog = false;
// Write per-request OBD/ISO-TP trace lines to the flash log (debug only).
constexpr bool PersistTraceToDiagnosticLog = false;
constexpr std::size_t DiagnosticLogMaxBytes = 64U * 1024U;

constexpr const char* SenderDiagnosticLogPath = "/sender_diagnostic.log";
constexpr const char* SenderDiagnosticLogArchivePath = "/sender_diagnostic.old";
constexpr const char* DisplayDiagnosticLogPath = "/display_diagnostic.log";
constexpr const char* DisplayDiagnosticLogArchivePath = "/display_diagnostic.old";

constexpr uint32_t TwaiStatusLogIntervalMs = 5000;

}
