#pragma once

#if defined(ARDUINO)
  #include <Arduino.h>
#else
  #include <string>
  using String = std::string;
#endif

namespace Network {

class WifiStationManager {
public:
    static void begin();
    static bool connect();
    static void handle();
    static void disconnect();
    static bool connected();
    static bool connecting();
    static String ip();
    static String ssid();
    static String statusJson();
    static bool configure(const String& ssid, const String& password);
    static bool forget();
};

} // namespace Network
