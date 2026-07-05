#include "WifiCredentialStore.h"

#include "config/NetworkConfig.h"

#if defined(ARDUINO)
  #include <Preferences.h>
#endif

namespace Network {

WifiCredentials WifiCredentialStore::load() {
    WifiCredentials credentials;
#if defined(ARDUINO)
    Preferences prefs;
    if (prefs.begin("canobd2-wifi", true)) {
        credentials.ssid = prefs.getString("ssid", "");
        credentials.password = prefs.getString("password", "");
        prefs.end();
        credentials.stored = credentials.ssid.length() > 0;
    }
#endif
    if (credentials.ssid.length() == 0 && NetworkConfig::WifiSsid[0] != '\0') {
        credentials.ssid = NetworkConfig::WifiSsid;
        credentials.password = NetworkConfig::WifiPassword;
        credentials.stored = false;
    }
    return credentials;
}

bool WifiCredentialStore::save(const String& ssid, const String& password) {
#if defined(ARDUINO)
    if (ssid.length() == 0) return false;
    Preferences prefs;
    if (!prefs.begin("canobd2-wifi", false)) return false;
    const bool ok = prefs.putString("ssid", ssid) > 0 &&
                    prefs.putString("password", password) == password.length();
    prefs.end();
    return ok;
#else
    (void)ssid;
    (void)password;
    return false;
#endif
}

bool WifiCredentialStore::clear() {
#if defined(ARDUINO)
    Preferences prefs;
    if (!prefs.begin("canobd2-wifi", false)) return false;
    const bool ok = prefs.clear();
    prefs.end();
    return ok;
#else
    return true;
#endif
}

} // namespace Network

