#include "OTAHandler.h"

namespace OTAHandler {

    void initOTA() {
        if (!Config::OtaEnable) return;

        Logger::serialPrintln("[OTA] Initialisierung...");

        // Hostname aus Config setzen
        ArduinoOTA.setHostname(Config::OtaHostname);

        // Passwort aus Config setzen (optional)
        if (strlen(Config::OtaPassword) > 0) {
            ArduinoOTA.setPassword(Config::OtaPassword);
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
        if (Config::OtaEnable) {
            ArduinoOTA.handle();
        }
    }
}