#include "WifiStationManager.h"

#include "WifiCredentialStore.h"
#include "config/NetworkConfig.h"

#if defined(ARDUINO)
  #include <WiFi.h>
#endif

namespace Network {
namespace {
String currentSsid;
String lastError;
uint32_t lastConnectAttemptMs = 0;
bool connectionInProgress = false;
int lastStationChannel = 0;

void applyStableWifiMode() {
#if defined(ARDUINO)
    WiFi.mode(WIFI_AP_STA);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
#endif
}

#if defined(ARDUINO)
bool acceptConnectedStationChannel() {
    lastStationChannel = WiFi.channel();
    if (NetworkConfig::DisconnectStationWifiOnEspNowChannelMismatch
        && lastStationChannel != 0
        && lastStationChannel != NetworkConfig::EspNowChannel) {
        lastError = "Hotspot-Kanal " + String(lastStationChannel)
                  + " passt nicht zu ESP-NOW-Kanal "
                  + String(NetworkConfig::EspNowChannel);
        WiFi.disconnect(false, false);
        return false;
    }
    lastError = "";
    return true;
}
#endif
}

void WifiStationManager::begin() {
    const auto credentials = WifiCredentialStore::load();
    currentSsid = credentials.ssid;
    applyStableWifiMode();
}

bool WifiStationManager::connect() {
#if defined(ARDUINO)
    if (!NetworkConfig::EnableStationWifi) {
        lastError = "Station-WLAN deaktiviert";
        return false;
    }
    const auto credentials = WifiCredentialStore::load();
    if (credentials.ssid.length() == 0) {
        lastError = "Keine WLAN-Zugangsdaten";
        return false;
    }
    if (WiFi.status() == WL_CONNECTED) {
        connectionInProgress = false;
        return acceptConnectedStationChannel();
    }
    if (connectionInProgress) {
        return true;
    }
    currentSsid = credentials.ssid;
    lastConnectAttemptMs = millis();
    connectionInProgress = true;
    lastError = "WLAN-Verbindung laeuft";
    applyStableWifiMode();
    WiFi.begin(credentials.ssid.c_str(), credentials.password.c_str());
    return true;
#else
    return false;
#endif
}

void WifiStationManager::handle() {
#if defined(ARDUINO)
    if (!connectionInProgress) return;

    if (WiFi.status() == WL_CONNECTED) {
        connectionInProgress = false;
        acceptConnectedStationChannel();
        return;
    }

    if (millis() - lastConnectAttemptMs >= NetworkConfig::WifiConnectTimeoutMs) {
        connectionInProgress = false;
        lastError = "WLAN-Verbindung Timeout";
        WiFi.disconnect(false, false);
    }
#endif
}

void WifiStationManager::disconnect() {
    connectionInProgress = false;
    lastStationChannel = 0;
#if defined(ARDUINO)
    WiFi.disconnect(false, false);
#endif
}

bool WifiStationManager::connected() {
#if defined(ARDUINO)
    return WiFi.status() == WL_CONNECTED;
#else
    return false;
#endif
}

bool WifiStationManager::connecting() {
    return connectionInProgress;
}

String WifiStationManager::ip() {
#if defined(ARDUINO)
    return connected() ? WiFi.localIP().toString() : String("");
#else
    return {};
#endif
}

String WifiStationManager::ssid() {
#if defined(ARDUINO)
    if (connected()) return WiFi.SSID();
#endif
    return currentSsid;
}

bool WifiStationManager::configure(const String& ssidValue, const String& password) {
    const bool ok = WifiCredentialStore::save(ssidValue, password);
    if (ok) currentSsid = ssidValue;
    return ok;
}

bool WifiStationManager::forget() {
    currentSsid = "";
    disconnect();
    return WifiCredentialStore::clear();
}

String WifiStationManager::statusJson() {
#if defined(ARDUINO)
    if (connectionInProgress) {
        handle();
    }
#endif
    String json = "{";
    json += "\"enabled\":";
    json += NetworkConfig::EnableStationWifi ? "true" : "false";
    json += ",\"connected\":";
    json += connected() ? "true" : "false";
    json += ",\"connecting\":";
    json += connecting() ? "true" : "false";
    json += ",\"ssid\":\"";
    json += ssid();
    json += "\",\"ip\":\"";
    json += ip();
    json += "\",\"lastError\":\"";
    json += lastError;
    json += "\",\"lastConnectAttemptMs\":";
#if defined(ARDUINO)
    json += String(static_cast<unsigned long>(lastConnectAttemptMs));
    const int currentChannel = connected() ? WiFi.channel() : lastStationChannel;
#else
    json += std::to_string(static_cast<unsigned long>(lastConnectAttemptMs));
    const int currentChannel = 0;
#endif
    json += ",\"stationChannel\":";
#if defined(ARDUINO)
    json += String(currentChannel);
#else
    json += std::to_string(currentChannel);
#endif
    json += ",\"espNowChannel\":";
#if defined(ARDUINO)
    json += String(NetworkConfig::EspNowChannel);
#else
    json += std::to_string(NetworkConfig::EspNowChannel);
#endif
    json += ",\"channelMatchesEspNow\":";
    json += (currentChannel == 0 || currentChannel == NetworkConfig::EspNowChannel) ? "true" : "false";
    json += "}";
    return json;
}

} // namespace Network
