#include "SenderEspNow.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

#include "Logger.h"
#include "AuthHelpers.h"
#include "config/NetworkConfig.h"

namespace {
bool readyState = false;
int meshIdState = 0;
uint8_t peerAddress[6] = {};
esp_now_peer_info_t peerInfo = {};

int computeMeshId() {
    uint8_t baseMac[6];
    WiFi.macAddress(baseMac);

    uint32_t uniqueId = 0;
    for (int index = 2; index < 6; ++index) {
        uniqueId <<= 8;
        uniqueId |= baseMac[index];
    }
    return static_cast<int>(uniqueId % 32768);
}
} // namespace

namespace SenderEspNow {

bool begin() {
    readyState = false;
    Logger::debug("[ESP-NOW] Initialisiere Sender-Transport");

    const String securityWarning = WebSecurity::espNowSecurityWarning();
    if (SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets && securityWarning.length() > 0) {
        Logger::alarm(("[ESP-NOW] Start blockiert: " + securityWarning).c_str());
        return false;
    }

    // Keep the management SoftAP alive while ESP-NOW is active. Switching back
    // to WIFI_STA here can make the sender web network disappear after boot.
    WiFi.mode(WIFI_AP_STA);
    WiFi.disconnect(false, false);

    if (esp_now_init() != ESP_OK) {
        Logger::alarm("[ESP-NOW] Init fehlgeschlagen");
        return false;
    }

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerAddress, NetworkConfig::DisplayPeerMac, sizeof(peerAddress));
    memcpy(peerInfo.peer_addr, peerAddress, sizeof(peerAddress));
    peerInfo.channel = NetworkConfig::EspNowChannel;
    peerInfo.encrypt = NetworkConfig::UseEspNowEncryption;

    if (NetworkConfig::UseEspNowEncryption) {
        memcpy(peerInfo.lmk, NetworkConfig::EspNowAesKey, sizeof(peerInfo.lmk));
    }

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Logger::alarm("[ESP-NOW] Peer konnte nicht hinzugefuegt werden");
        return false;
    }

    meshIdState = computeMeshId();
    readyState = true;
    Logger::debugf("[ESP-NOW] Peer OK, meshId=%d", meshIdState);
    return true;
}

bool ready() {
    return readyState;
}

int meshId() {
    return meshIdState;
}

} // namespace SenderEspNow
