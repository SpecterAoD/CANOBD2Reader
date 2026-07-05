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
}

void WifiStationManager::begin() {
#if defined(ARDUINO)
    WiFi.mode(WIFI_AP_STA);
#endif
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
    currentSsid = credentials.ssid;
    lastConnectAttemptMs = millis();
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(credentials.ssid.c_str(), credentials.password.c_str());
    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < NetworkConfig::WifiConnectTimeoutMs) {
        delay(100);
        yield();
    }
    if (WiFi.status() == WL_CONNECTED) {
        lastError = "";
        return true;
    }
    lastError = "WLAN-Verbindung fehlgeschlagen";
    return false;
#else
    return false;
#endif
}

void WifiStationManager::disconnect() {
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
    String json = "{";
    json += "\"enabled\":";
    json += NetworkConfig::EnableStationWifi ? "true" : "false";
    json += ",\"connected\":";
    json += connected() ? "true" : "false";
    json += ",\"ssid\":\"";
    json += ssid();
    json += "\",\"ip\":\"";
    json += ip();
    json += "\",\"lastError\":\"";
    json += lastError;
    json += "\",\"lastConnectAttemptMs\":";
#if defined(ARDUINO)
    json += String(static_cast<unsigned long>(lastConnectAttemptMs));
#else
    json += std::to_string(static_cast<unsigned long>(lastConnectAttemptMs));
#endif
    json += "}";
    return json;
}

} // namespace Network
