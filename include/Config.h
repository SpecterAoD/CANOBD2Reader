#pragma once
#include <Arduino.h>
#include "PIDs.h"

namespace Config {

    // ----------- Firmware -----------
    // constexpr const char* FirmwareVersion = "3.1203";
    constexpr const char* FirmwareVersion = "1.0.0";
    // ===== WebConsole Settings =====
    inline bool EnableWebConsole = true;      // Aktiviert / deaktiviert Web-Konsole
    inline const char* DebugAPSSID = "ESP_OBD_Debug";
    inline const char* DebugAPPass = "12345678";
    inline uint16_t WebConsolePort = 80;
    inline size_t WebConsoleMaxLines = 50;    // Ringpuffergröße für Logs

    // Start-Button: Muss gedrückt werden, bevor Init fortfährt
    inline bool RequireWebStart = true;

    // ----------- Debug Flags --------
    constexpr bool DebugFlag       = true;  // Serielle Debug-Ausgaben
    constexpr bool TwaiDebugFlag   = true;  // TWAI/CAN Statusmeldungen
    constexpr bool PowerDebugFlag  = true;  // Batteriespannung & Power
    constexpr bool CanDebugFlag = true;
    constexpr bool OBD2DebugFlag = true;
    constexpr bool SerialFlag      = true;   // Serielle Ausgabe generell

    // ----------- Serial Settings ----
    constexpr int Baudrate = 115200;

    // ----------- CAN / OBD2 ---------
    constexpr bool EnableCAN    = true;  
    constexpr bool EnableOBD2   = true;
    constexpr bool sendRAWData = false;

    constexpr int PollingRateMs = 500;
    constexpr int CanIdleTimeout = 500;
    constexpr int SleepPeriodSec = 6;
    constexpr bool sendRawDataOnly = false;
    constexpr uint32_t ObdIntervalMs = 200;
    //constexpr const char* TWAI_OPERATION_MODE = "TWAI_MODE_LISTEN";
    constexpr const char* TWAI_OPERATION_MODE = "TWAI_MODE_NORMAL";

    // ----------- OBD2 PIDs ----------
    inline constexpr byte ObdRequestedPids[] = {
        ENGINE_RPM,
        ENGINE_COOLANT_TEMP,
        ENGINE_OIL_TEMP,
        VEHICLE_SPEED,
        ENGINE_FUEL_RATE
    };

    inline constexpr size_t ObdPidCount =
        sizeof(ObdRequestedPids) / sizeof(ObdRequestedPids[0]);

    // ----------- Pins ---------------
    constexpr int LedPin1 = 26;
    constexpr int LedPin2 = 27;
    constexpr int ButtonPin = 25;
    constexpr int SHIELD_CAN_RX = 4;
    constexpr int SHIELD_CAN_TX = 5;
    constexpr int VoltageDividerPin = 32;

    // ----------- Power --------------
    constexpr uint32_t StartStopDelayMs = 300000; // 5 Min
    constexpr int CpuFrequency = 80;              // MHz im Sleep
    constexpr float VoltageCalcFactor = 4.81f;
    constexpr float VoltageChangeThreshold = 0.2f;

    // ----------- WiFi ---------------
    constexpr const char* WifiSsid     = "MeinOBDNetz";
    constexpr const char* WifiPassword = "GeheimesPasswort123";
    constexpr bool WifiEnable          = true;

    // ----------- Bluetooth -----------
    constexpr const char* BluetoothName = "ESP-BT";
    constexpr bool BluetoothEnable = false;

    // ----------- OTA ----------------
    constexpr const char* OtaHostname  = "CAN_OBD2_Gateway";
    constexpr const char* OtaPassword  = "Update123"; // optional
    constexpr bool OtaEnable           = true;
    constexpr uint32_t UpdateIntervalMs = 10000;

    // --- Update / GitHub ---
    constexpr const char* UpdateCheckUrl =
        "https://raw.githubusercontent.com/user/repo/main/firmware.bin";
    constexpr const char* FirmwareBinUrl =
        "https://raw.githubusercontent.com/user/repo/main/firmware.bin";
    // ----------- ESP-NOW ------------
    constexpr bool UseEspNowEncryption = true;
    constexpr uint8_t EspNowAesKey[16] = {
        0x3A, 0x7F, 0xC2, 0x91, 0x18, 0x5D, 0xE0, 0xB3,
        0x4C, 0x22, 0xA1, 0x6E, 0xD4, 0x0F, 0x97, 0x8B
    };
    constexpr uint8_t EspNowPeerMac[6] = { 0xF0, 0xF5, 0xBD, 0x43, 0x29, 0x20 };

    // ===================== Structs =====================
    struct VoltageMeasurement {
        int adcRaw = 0;
        float voltage = 0.0f;
    };

    struct EspNowTextFrame {
        char payload[128]{};
        uint16_t crc = 0;
    };

    struct EspNowCanFrame {
        uint8_t magic = 0x42;
        uint32_t timestamp = 0;
        int meshId = 0;
        uint32_t canId = 0;
        uint8_t len = 0;
        uint8_t data[8]{};
        uint16_t crc = 0;
    };

    // ===================== Globale Variablen =====================
    inline uint32_t currentMillis = 0;
    inline uint32_t lastCanMsgTimestamp = 0;
    inline uint32_t lastObdRequestTime = 0;
    inline uint32_t lastCarRunningTime = 0;
    inline uint32_t lastBatterySendTime = 0;
    inline uint32_t ledTestStartTime = 0;
    inline uint32_t lastSpeed = 0;
    inline uint32_t lastFuelRate = 0;
    inline float consumption = 0.0f;
    inline float consumptionSum = 0.0f;
    inline int consumptionCount = 0;


    inline bool carIsRunning = false;
    inline bool ledTestActive = false;
    inline uint8_t ledTestStep = 0;

    inline EspNowTextFrame textFrame;
    inline EspNowCanFrame  canFrame;
}