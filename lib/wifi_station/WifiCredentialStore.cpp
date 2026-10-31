#include "WifiCredentialStore.h"

#include "DeviceSecrets.h"

#if defined(ARDUINO)
  #include <Preferences.h>
#endif

namespace WifiNet {

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
    // Fallback: optional station WiFi from the provisioned device secrets.
    const Security::DeviceSecrets& secrets = Security::secrets();
    if (credentials.ssid.length() == 0 && secrets.stationSsid[0] != '\0') {
        credentials.ssid = secrets.stationSsid;
        credentials.password = secrets.stationPassword;
        credentials.stored = false;
    }
    return credentials;
}

bool WifiCredentialStore::save(const String& ssid, const String& password) {
#if defined(ARDUINO)
    if (ssid.length() == 0) return false;
    Preferences prefs;
    if (!prefs.begin("canobd2-wifi", false)) return false;
    // putString() returns 0 for an empty string, which is valid for open networks.
    bool ok = prefs.putString("ssid", ssid) > 0;
    if (password.length() == 0) {
        prefs.remove("password");
    } else {
        ok = ok && prefs.putString("password", password) == password.length();
    }
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

} // namespace WifiNet

