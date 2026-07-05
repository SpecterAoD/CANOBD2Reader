#pragma once

#if defined(ARDUINO)
  #include <Arduino.h>
#else
  #include <string>
  using String = std::string;
#endif

namespace Network {

struct WifiCredentials {
    String ssid;
    String password;
    bool stored = false;
};

class WifiCredentialStore {
public:
    static WifiCredentials load();
    static bool save(const String& ssid, const String& password);
    static bool clear();
};

} // namespace Network

