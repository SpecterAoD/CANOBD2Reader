#include "OTAHandler.h"

#include "AuthHelpers.h"

namespace OTAHandler {

    void initOTA() {
        if (!SenderConfig::EnableSenderOta) return;

        const String securityWarning = WebSecurity::senderManagementSecurityWarning();
        if (SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets && securityWarning.length() > 0) {
            Logger::alarm(("[OTA] Start blockiert: " + securityWarning).c_str());
            return;
        }

        Logger::debug("[OTA] Initialisierung...");

        if (WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP && WiFi.getMode() != WIFI_AP_STA) {
            WiFi.mode(WIFI_AP_STA);
            WiFi.softAP(NetworkConfig::SenderWebSsid,
                        NetworkConfig::SenderWebPassword,
                        NetworkConfig::EspNowChannel);
            Logger::debugf("[OTA] SoftAP gestartet, IP: %s", WiFi.softAPIP().toString().c_str());
        }

        // Hostname aus Config setzen
        ArduinoOTA.setHostname(NetworkConfig::SenderOtaHostname);

        // Passwort aus Config setzen (optional)
        if (strlen(NetworkConfig::SenderOtaPassword) > 0) {
            ArduinoOTA.setPassword(NetworkConfig::SenderOtaPassword);
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
                if (total == 0) return;
                char buf[64];
                snprintf(buf, sizeof(buf), "[OTA] Fortschritt: %u%%", (progress * 100U) / total);
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
        if (SenderConfig::EnableSenderOta) {
            ArduinoOTA.handle();
        }
    }
}
