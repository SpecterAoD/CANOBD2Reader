#include "EspNowTelemetryTransport.h"

#if defined(ARDUINO)
#include "config/NetworkConfig.h"
#include "DiagnosticLog.h"
#endif

namespace Transport {

esp_err_t sendTelemetryPacket(const Telemetry::TelemetryPacket& packet) {
#if defined(ARDUINO)
    return esp_now_send(NetworkConfig::DisplayPeerMac,
                        reinterpret_cast<const uint8_t*>(&packet),
                        sizeof(packet));
#else
    (void)packet;
    return ESP_OK;
#endif
}

void logTelemetrySendResult(esp_err_t result, uint32_t sequence, const char* payload) {
#if defined(ARDUINO)
    const char* safePayload = payload == nullptr ? "" : payload;
    if (result == ESP_OK) {
        DiagnosticLog::appendf("[TX] seq=%lu %s",
                               static_cast<unsigned long>(sequence),
                               safePayload);
    } else {
        DiagnosticLog::appendf("[TX] failed err=%d seq=%lu %s",
                               static_cast<int>(result),
                               static_cast<unsigned long>(sequence),
                               safePayload);
    }
#else
    (void)result;
    (void)sequence;
    (void)payload;
#endif
}

}
