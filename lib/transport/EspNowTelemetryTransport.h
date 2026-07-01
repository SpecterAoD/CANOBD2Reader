#pragma once

#include <cstdint>
#include "TelemetryPacket.h"

#if defined(ARDUINO)
#include <esp_now.h>
#else
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
#endif

namespace Transport {

/// Sends one already encoded telemetry packet to the configured display peer.
/// Keeping this tiny wrapper central avoids hidden esp_now_send call sites.
esp_err_t sendTelemetryPacket(const Telemetry::TelemetryPacket& packet);

/// Shared diagnostic formatter for successful and failed telemetry sends.
void logTelemetrySendResult(esp_err_t result, uint32_t sequence, const char* payload);

}
