#include "OTAHandler.h"

namespace OTAHandler {

    void initOTA() {
        if (!SenderLegacyConfig::EnableSenderOta) return;

        Logger::debug("[OTA] Initialisierung...");

        // Hostname aus Config setzen
        ArduinoOTA.setHostname(NetworkLegacyConfig::SenderOtaHostname);

        // Passwort aus Config setzen (optional)
        if (strlen(NetworkLegacyConfig::SenderOtaPassword) > 0) {
            ArduinoOTA.setPassword(NetworkLegacyConfig::SenderOtaPassword);
        }

        // Event-Handler
        ArduinoOTA
            .onStart([]() {
                Logger::alarm("[OTA] Update gestartet!");
            })
            .onEnd([]() {
                Logger::alarm("[OTA] Update beendet.");
            })
            .onProgress([](unsigned int progress, unsigned int total) {
                char buf[64];
                snprintf(buf, sizeof(buf), "[OTA] Fortschritt: %u%%", (progress / (total / 100)));
                Logger::debug(buf);
            })
            .onError([](ota_error_t error) {
                char buf[64];
                snprintf(buf, sizeof(buf), "[OTA] Fehler: %u", error);
                Logger::alarm(buf);
            });

        ArduinoOTA.begin();
        Logger::debug("[OTA] Bereit für Updates.");
    }

    void handleOTA() {
        if (SenderLegacyConfig::EnableSenderOta) {
            ArduinoOTA.handle();
        }
    }
}