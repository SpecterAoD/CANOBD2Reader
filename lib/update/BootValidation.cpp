#include "BootValidation.h"

#if defined(ARDUINO)
  #include <esp_ota_ops.h>
  #include "DiagnosticLog.h"

// Arduino-ESP32 calls this weak hook during startup; returning true keeps a
// new image in ESP_OTA_IMG_PENDING_VERIFY instead of confirming it at once.
extern "C" bool verifyRollbackLater() {
    return true;
}
#endif

namespace BootValidation {
namespace {
bool done = false;
HealthGate health;
} // namespace

bool pendingVerify() {
#if defined(ARDUINO)
    esp_ota_img_states_t state;
    const esp_partition_t* running = esp_ota_get_running_partition();
    return running != nullptr && esp_ota_get_state_partition(running, &state) == ESP_OK &&
           state == ESP_OTA_IMG_PENDING_VERIFY;
#else
    return false;
#endif
}

void tick(uint32_t nowMs, bool localHealthy) {
    if (done) return;
    const Decision decision = health.evaluate(nowMs, localHealthy);
    if (decision == Decision::Wait) return;
#if defined(ARDUINO)
    if (pendingVerify()) {
        if (decision == Decision::Rollback) {
            DiagnosticLog::appendf("[OTA] Lokale Initialisierung fehlgeschlagen, Rollback");
            // If no previous valid slot exists, remain in the recovery UI.
            const esp_err_t result = esp_ota_mark_app_invalid_rollback_and_reboot();
            DiagnosticLog::appendf("[OTA] Rollback nicht moeglich: %d", static_cast<int>(result));
            done = true;
            return;
        }
        const bool ok = esp_ota_mark_app_valid_cancel_rollback() == ESP_OK;
        DiagnosticLog::appendf(ok ? "[OTA] Neue Firmware stabil, bestaetigt" : "[OTA] Bestaetigung fehlgeschlagen");
        // A failed confirmation must not be silently considered successful.
        if (!ok) return;
    }
#endif
    done = true;
}

} // namespace BootValidation
