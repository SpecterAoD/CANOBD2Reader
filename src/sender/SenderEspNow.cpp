#include "SenderEspNow.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <string.h>

#include "Logger.h"
#include "SenderTelemetry.h"
#include "DeviceSecrets.h"
#include "config/NetworkConfig.h"

namespace {
bool readyState = false;
int meshIdState = 0;
uint8_t peerAddress[6] = {};
esp_now_peer_info_t peerInfo = {};

// MAC-level acknowledgement of each unicast packet (WiFi task context).
void onSent(const esp_now_send_info_t*, esp_now_send_status_t status) {
    SenderTelemetry::onDelivery(status == ESP_NOW_SEND_SUCCESS);
}

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

    // Never run the link with a publicly known key or without a peer MAC.
    if (!Security::provisioned()) {
        Logger::alarm((String("[ESP-NOW] Start blockiert: ") + Security::problemText()).c_str());
        return false;
    }
    const Security::DeviceSecrets& secrets = Security::secrets();

    // Keep the management SoftAP alive while ESP-NOW is active. Switching back
    // to WIFI_STA here can make the sender web network disappear after boot.
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(false, false);

    if (esp_now_init() != ESP_OK) {
        Logger::alarm("[ESP-NOW] Init fehlgeschlagen");
        return false;
    }

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerAddress, secrets.displayMac, sizeof(peerAddress));
    memcpy(peerInfo.peer_addr, peerAddress, sizeof(peerAddress));
    peerInfo.channel = NetworkConfig::EspNowChannel;
    peerInfo.encrypt = NetworkConfig::UseEspNowEncryption;

    if (NetworkConfig::UseEspNowEncryption) {
        memcpy(peerInfo.lmk, secrets.espNowKey, sizeof(peerInfo.lmk));
    }

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Logger::alarm("[ESP-NOW] Peer konnte nicht hinzugefuegt werden");
        return false;
    }

    esp_now_register_send_cb(onSent);
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
