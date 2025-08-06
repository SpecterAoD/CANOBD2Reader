#include "handlers/NetworkManager.h"

namespace NetworkManager {

    // ESP-NOW Peer Info
    inline esp_now_peer_info_t peerInfo = {};
    inline BluetoothSerial btSerial;

    // ============= WiFi =============
    bool initWiFi(const char* ssid, const char* password) {
        Logger::debug("[WiFi] Initialisiere WiFi...");

        WiFi.mode(WIFI_STA);
        if (ssid && password) {
            WiFi.begin(ssid, password);
            Logger::debugf("[WiFi] Verbinde mit SSID: %s", ssid);

            unsigned long startAttempt = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
                delay(500);
                Serial.print(".");
            }

            if (WiFi.status() == WL_CONNECTED) {
                Logger::debugf("[WiFi] Verbunden, IP: %s", WiFi.localIP().toString().c_str());
                return true;
            } else {
                Logger::alarm("[WiFi] Verbindung fehlgeschlagen!");
                return false;
            }
        } else {
            Logger::debug("[WiFi] Kein SSID/Passwort, nur Station-Mode aktiviert.");
            return true;
        }
    }

    bool isWiFiConnected() {
        return WiFi.status() == WL_CONNECTED;
    }

    // ============= ESP-NOW =============
    bool initEspNow() {
        Logger::debug("[ESP-NOW] Initialisierung...");

        WiFi.mode(WIFI_STA);
        WiFi.disconnect();

        if (esp_now_init() != ESP_OK) {
            Logger::alarm("[ESP-NOW] Init fehlgeschlagen!");
            return false;
        }

        esp_now_register_send_cb(onDataSent);
        esp_now_register_recv_cb(onDataRecv);

        memset(&peerInfo, 0, sizeof(peerInfo));
        memcpy(peerInfo.peer_addr, Config::EspNowPeerMac, 6);
        peerInfo.channel = 1;
        peerInfo.encrypt = Config::UseEspNowEncryption;

        if (Config::UseEspNowEncryption) {
            memcpy(peerInfo.lmk, Config::EspNowAesKey, 16);
        }

        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Logger::alarm("[ESP-NOW] Peer hinzufügen fehlgeschlagen!");
            return false;
        }

        Logger::debug("[ESP-NOW] Peer erfolgreich hinzugefügt");
        return true;
    }

    bool sendEspNow(const uint8_t* data, size_t length) {
        esp_err_t result = esp_now_send(peerInfo.peer_addr, data, length);
        if (result == ESP_OK) {
            Logger::twai("[ESP-NOW] Nachricht gesendet");
            return true;
        } else {
            Logger::alarm("[ESP-NOW] Senden fehlgeschlagen!");
            return false;
        }
    }

    void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
        if (status == ESP_NOW_SEND_SUCCESS) {
            Logger::debug("[ESP-NOW] Senden OK");
        } else {
            Logger::alarm("[ESP-NOW] Senden fehlgeschlagen");
        }
    }

    void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
        Logger::debugf("[ESP-NOW] Empfangene Daten: %d Bytes", len);
    }

    // ============= Bluetooth =============
    bool initBluetooth(const String& deviceName) {
        Logger::debugf("[BT] Starte Bluetooth mit Name: %s", deviceName.c_str());

        if (!btSerial.begin(deviceName)) {
            Logger::alarm("[BT] Init fehlgeschlagen!");
            return false;
        }

        Logger::debug("[BT] Bluetooth bereit");
        return true;
    }

    bool isBluetoothConnected() {
        return btSerial.hasClient();
    }

    bool sendBluetooth(const String& msg) {
        if (!isBluetoothConnected()) return false;
        btSerial.println(msg);
        Logger::debug("[BT] Nachricht gesendet");
        return true;
    }

    // ============= Platzhalter =============
    bool sendWiFiTCP(const uint8_t* data, size_t length) {
        // Optional: Später implementieren
        return false;
    }

    void loop() {
        // Optional: Status überwachen oder Auto-Reconnect implementieren
    }
}