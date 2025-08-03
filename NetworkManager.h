#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <BluetoothSerial.h>

#include "Config.h"
#include "Logger.h"

namespace NetworkManager {

    // ==================== Typen ====================
    enum class NetworkMode {
        WiFi,
        EspNow,
        Bluetooth
    };

    // ==================== Initialisierung ====================
    bool initWiFi(const char* ssid = nullptr, const char* password = nullptr);
    bool initEspNow();
    bool initBluetooth(const String& deviceName = "ESP32_BT");

    // ==================== Daten senden ====================
    bool sendEspNow(const uint8_t* data, size_t length);
    bool sendWiFiTCP(const uint8_t* data, size_t length); // Optional: TCP/UDP später
    bool sendBluetooth(const String& msg);

    // ==================== Callback-Handler ====================
    void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status);
    void onDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len);

    // ==================== Status & Hilfsfunktionen ====================
    bool isWiFiConnected();
    bool isBluetoothConnected();
    void loop(); // Optional für zukünftige Status-Checks
}