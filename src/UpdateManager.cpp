#include "UpdateManager.h"

namespace UpdateManager {

    static unsigned long lastUpdateCheck = 0;

    // ============ OTA Initialisierung ============
    void initOTA() {
        if (!Config::OtaEnable) return;

        ArduinoOTA.setHostname(Config::OtaHostname);
        ArduinoOTA.setPassword(Config::OtaPassword);

        ArduinoOTA
            .onStart([]() { Logger::alarm("[OTA] Update gestartet"); })
            .onEnd([]() { Logger::alarm("[OTA] Update abgeschlossen"); })
            .onProgress([](unsigned int progress, unsigned int total) {
                Serial.printf("OTA Fortschritt: %u%%\r", (progress / (total / 100)));
            })
            .onError([](ota_error_t error) {
                Serial.printf("[OTA] Fehler: %u\n", error);
            });

        ArduinoOTA.begin();
        Logger::alarm("[OTA] Ready, Host: ");
        Logger::debug(Config::OtaHostname);
    }

    void handleOTA() {
        if (Config::OtaEnable) {
            ArduinoOTA.handle();
        }
    }

    // ============ Hotspot verbinden ============
    bool connectHotspot() {
        Logger::debug("[Update] Verbindung mit Hotspot...");
        WiFi.mode(WIFI_STA);
        WiFi.begin(Config::WifiSSID, Config::WifiPassword);

        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
            delay(500);
            Serial.print(".");
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
            Logger::debug("[Update] Hotspot verbunden!");
            return true;
        } else {
            Logger::debug("[Update] Hotspot Verbindung fehlgeschlagen.");
            return false;
        }
    }

    // ============ Prüfen auf neue Version ============
    bool checkForUpdate() {
        if (!Config::OtaEnable) return false;

        unsigned long now = millis();
        if (now - lastUpdateCheck < Config::UpdateIntervalMs) {
            return false; // Noch nicht Zeit
        }
        lastUpdateCheck = now;

        if (!connectHotspot()) return false;

        HTTPClient http;
        http.begin(Config::UpdateCheckUrl);

        int httpCode = http.GET();
        if (httpCode != 200) {
            Logger::debug("[Update] Konnte Version nicht abrufen.");
            http.end();
            WiFi.disconnect(true);
            return false;
        }

        String newVersion = http.getString();
        newVersion.trim();
        http.end();

        Logger::debug("[Update] Aktuelle FW: ");
        Logger::debug(Config::FirmwareVersion);
        Logger::debug("[Update] Online FW: ");
        Logger::debug(newVersion);

        if (newVersion != Config::FirmwareVersion) {
            Logger::alarm("[Update] Neue Version gefunden!");
            return downloadAndUpdate();
        } else {
            Logger::debug("[Update] Firmware aktuell.");
        }

        WiFi.disconnect(true);
        return false;
    }

    // ============ Firmware herunterladen & aktualisieren ============
    bool downloadAndUpdate() {
        HTTPClient http;
        http.begin(Config::FirmwareBinUrl);
        int httpCode = http.GET();

        if (httpCode != 200) {
            Logger::alarm("[Update] Firmware Download fehlgeschlagen!");
            http.end();
            WiFi.disconnect(true);
            return false;
        }

        int contentLength = http.getSize();
        if (contentLength <= 0) {
            Logger::alarm("[Update] Ungültige Firmwaregröße!");
            http.end();
            return false;
        }

        WiFiClient* stream = http.getStreamPtr();
        if (!Update.begin(contentLength)) {
            Logger::alarm("[Update] Update.begin() fehlgeschlagen!");
            http.end();
            return false;
        }

        Logger::alarm("[Update] Firmware wird heruntergeladen...");
        size_t written = Update.writeStream(*stream);

        if (written == contentLength && Update.end()) {
            Logger::alarm("[Update] Update erfolgreich. Neustart...");
            ESP.restart();
        } else {
            Logger::alarm("[Update] Update fehlgeschlagen!");
        }

        http.end();
        WiFi.disconnect(true);
        return true;
    }
}