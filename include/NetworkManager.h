#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "common_config.h"
#include "Config.h"
#include "Logger.h"

#if CANOBD2_ENABLE_BLUETOOTH
#include <BluetoothSerial.h>
#endif

namespace NetworkManager {

    enum class NetworkMode {
        WiFi,
        EspNow,
        Bluetooth
    };

    bool initWiFi(const char* ssid = nullptr, const char* password = nullptr);
    bool initEspNow();

#if CANOBD2_ENABLE_BLUETOOTH
    bool initBluetooth(const String& deviceName = Config::Network::BluetoothName);
#endif

    /// @brief Startet einen WiFi-Access-Point für WebConsole / Debug-Modus.
    void startAccessPoint(const char* ssid = Config::Network::SenderWebSsid,
                          const char* password = Config::Network::SenderWebPassword);

    bool sendEspNow(const uint8_t* data, size_t length);
    bool sendWiFiTCP(const uint8_t* data, size_t length);

#if CANOBD2_ENABLE_BLUETOOTH
    bool sendBluetooth(const String& msg);
#endif

    void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
    void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len);

    bool isWiFiConnected();

#if CANOBD2_ENABLE_BLUETOOTH
    bool isBluetoothConnected();
#endif

    void loop();
}
