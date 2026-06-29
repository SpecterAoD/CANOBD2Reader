#pragma once

#include <Arduino.h>
#include <driver/twai.h>

#include "common_config.h"
#include "PIDs.h"

namespace Config {

    // =========================================================================
    // Projekt / Firmware
    // =========================================================================
    namespace Project {
        constexpr const char* FirmwareVersion = CANOBD2_FIRMWARE_VERSION;
        constexpr uint8_t ProtocolVersion = CANOBD2_PROTOCOL_VERSION;
        constexpr const char* TargetName = CANOBD2_TARGET_NAME;
        constexpr int Baudrate = 115200;
    }

    // =========================================================================
    // Feature Flags
    // =========================================================================
    namespace Feature {
        constexpr bool EnableBluetooth = CANOBD2_ENABLE_BLUETOOTH;
        constexpr bool EnableSenderWebConsole = CANOBD2_ENABLE_SENDER_WEBCONSOLE;
        constexpr bool EnableDisplayWebOta = CANOBD2_ENABLE_DISPLAY_OTA;
        constexpr bool EnableSenderOta = true;

        // Sender-Simulation: sendet ohne Fahrzeug/CAN-Transceiver Testwerte.
        constexpr bool EnableSenderTelemetrySimulation = false;

        // Display-Simulation: erzeugt lokal Testwerte fuer alle Display-Seiten.
        constexpr bool EnableDisplayInternalSimulation = false;
    }

    // =========================================================================
    // Netzwerk, Weboberflaechen, OTA und ESP-NOW
    // =========================================================================
    namespace Network {
        constexpr const char* BluetoothName = "ESP-BT";

        // WebConsole / OTA Access Point des Senders.
        constexpr const char* SenderWebSsid = "ESP_OBD_Debug";
        constexpr const char* SenderWebPassword = "12345678";
        constexpr const char* SenderOtaHostname = "CAN_OBD2_Gateway";
        constexpr const char* SenderOtaPassword = "Update123";

        // Web-OTA Access Point des Displays.
        constexpr const char* DisplayWebSsid = "CANOBD2_Display_OTA";
        constexpr const char* DisplayWebPassword = "Update123";
        constexpr const char* DisplayOtaHostname = "CANOBD2_Display";

        constexpr uint16_t WebServerPort = 80;
        constexpr size_t WebConsoleMaxLines = 50;
        constexpr bool RequireWebStart = true;

        // Optionales stationaeres WLAN fuer spaetere Erweiterungen.
        constexpr const char* WifiSsid = "MeinOBDNetz";
        constexpr const char* WifiPassword = "GeheimesPasswort123";
        constexpr bool WifiEnable = true;

        // ESP-NOW: Sender sendet an die Display-MAC; Display akzeptiert nur die
        // Sender-MAC. Beide Werte muessen zu den realen Geraeten passen.
        constexpr uint8_t EspNowChannel = 1;
        constexpr bool UseEspNowEncryption = true;
        constexpr uint8_t EspNowAesKey[16] = {
            0x3A, 0x7F, 0xC2, 0x91, 0x18, 0x5D, 0xE0, 0xB3,
            0x4C, 0x22, 0xA1, 0x6E, 0xD4, 0x0F, 0x97, 0x8B
        };
        constexpr uint8_t DisplayPeerMac[6] = { 0xF0, 0xF5, 0xBD, 0x43, 0x29, 0x20 };
        constexpr uint8_t SenderAllowedMac[6] = { 0x8C, 0x4B, 0x14, 0x27, 0xEB, 0x48 };
    }

    // =========================================================================
    // Display-Hardware und UI-Timing
    // =========================================================================
    namespace Display {
        constexpr uint8_t Rotation = 1;

        // LilyGO T-Display S3: Pin 15 schaltet die Displayversorgung frei.
        // Wenn eine Board-Revision diesen Pin nicht nutzt, auf -1 setzen.
        constexpr int8_t PowerPin = 15;
        constexpr uint8_t PowerOnLevel = HIGH;
        constexpr uint16_t PowerStabilizeMs = 60;

        constexpr uint8_t BacklightPin = 38;
        constexpr uint8_t BacklightOn = 255;
        constexpr uint8_t NextPageButtonPin = 0;
        constexpr uint8_t PageCount = 7;

        constexpr uint32_t ScreenRefreshMs = 120;
        constexpr uint32_t ForceFullRenderMs = 1200;
        constexpr bool UseSegmentValueRenderer = false;
        constexpr bool EnableStartupValueOverlay = false;
        constexpr uint32_t StartupValueOverlayMs = 10000;
        constexpr float SpeedSmoothingAlpha = 0.28f;
        constexpr float RpmSmoothingAlpha = 0.20f;
        constexpr uint32_t ConnectionTimeoutMs = 3000;
        constexpr uint32_t ValueTimeoutMs = 5000;
        constexpr uint32_t ButtonDebounceMs = 250;
    }

    // =========================================================================
    // Sender-Hardware, CAN/OBD2 und Power-Management
    // =========================================================================
    namespace Sender {
        constexpr bool EnableCAN = true;
        constexpr bool EnableOBD2 = true;
        constexpr bool SendRawData = false;
        constexpr bool SendRawDataOnly = false;

        constexpr int LedPin1 = 26;
        constexpr int LedPin2 = 27;
        constexpr int ButtonPin = 25;
        constexpr int ShieldCanRx = 4;
        constexpr int ShieldCanTx = 5;
        constexpr int VoltageDividerPin = 32;

        constexpr int PollingRateMs = 500;
        constexpr int CanIdleTimeoutMs = 500;
        constexpr int SleepPeriodSec = 6;
        constexpr uint32_t ObdIntervalMs = 200;
        constexpr uint32_t ObdResponseTimeoutMs = 250;
        constexpr uint32_t ObdTxTimeoutMs = 50;
        constexpr uint32_t BatterySendIntervalMs = 3000;
        constexpr uint32_t SimulationIntervalMs = 250;
        constexpr uint32_t SupportedPidRefreshIntervalMs = 60000;
        constexpr uint32_t DtcQueryIntervalMs = 30000;
        constexpr uint32_t StartStopDelayMs = 300000;
        constexpr int CpuFrequency = 80;

        constexpr float VoltageCalcFactor = 4.81f;
        constexpr float VoltageChangeThreshold = 0.2f;

        constexpr twai_mode_t TwaiOperationMode = TWAI_MODE_NORMAL;
    }

    // =========================================================================
    // Debug
    // =========================================================================
    namespace Debug {
        constexpr bool Serial = true;
        constexpr bool General = true;
        constexpr bool Twai = true;
        constexpr bool Power = true;
        constexpr bool Can = true;
        constexpr bool Obd2 = true;
        constexpr bool TraceSenderTelemetry = true;
        constexpr bool TraceDisplayTelemetry = true;
        constexpr uint32_t TraceSummaryIntervalMs = 1000;
    }

    // =========================================================================
    // OBD2 PIDs
    // =========================================================================
    inline constexpr byte ObdRequestedPids[] = {
        ENGINE_RPM,
        ENGINE_LOAD,
        ENGINE_COOLANT_TEMP,
        VEHICLE_SPEED,
        INTAKE_AIR_TEMP,
        MAF_FLOW_RATE,
        THROTTLE_POSITION,
        FUEL_TANK_LEVEL_INPUT,
        RUN_TIME_SINCE_ENGINE_START,
        CONTROL_MODULE_VOLTAGE,
        AMBIENT_AIR_TEMP,
        ENGINE_OIL_TEMP,
        ENGINE_FUEL_RATE
    };

    inline constexpr size_t ObdPidCount = sizeof(ObdRequestedPids) / sizeof(ObdRequestedPids[0]);

    // =========================================================================
    // Kompatibilitaets-Aliase fuer bestehende Module.
    // Neue Module sollen bevorzugt Config::<Bereich>::<Name> verwenden.
    // =========================================================================
    constexpr const char* FirmwareVersion = Project::FirmwareVersion;
    constexpr int Baudrate = Project::Baudrate;

    inline bool EnableWebConsole = Feature::EnableSenderWebConsole;
    inline const char* DebugAPSSID = Network::SenderWebSsid;
    inline const char* DebugAPPass = Network::SenderWebPassword;
    inline uint16_t WebConsolePort = Network::WebServerPort;
    inline size_t WebConsoleMaxLines = Network::WebConsoleMaxLines;
    inline bool RequireWebStart = Network::RequireWebStart;

    constexpr bool DebugFlag = Debug::General;
    constexpr bool TwaiDebugFlag = Debug::Twai;
    constexpr bool PowerDebugFlag = Debug::Power;
    constexpr bool CanDebugFlag = Debug::Can;
    constexpr bool OBD2DebugFlag = Debug::Obd2;
    constexpr bool SerialFlag = Debug::Serial;

    constexpr bool EnableCAN = Sender::EnableCAN;
    constexpr bool EnableOBD2 = Sender::EnableOBD2;
    constexpr bool sendRAWData = Sender::SendRawData;
    constexpr bool sendRawDataOnly = Sender::SendRawDataOnly;
    constexpr int PollingRateMs = Sender::PollingRateMs;
    constexpr int CanIdleTimeout = Sender::CanIdleTimeoutMs;
    constexpr int SleepPeriodSec = Sender::SleepPeriodSec;
    constexpr uint32_t ObdIntervalMs = Sender::ObdIntervalMs;
    constexpr uint32_t ObdResponseTimeoutMs = Sender::ObdResponseTimeoutMs;
    constexpr uint32_t ObdTxTimeoutMs = Sender::ObdTxTimeoutMs;
    constexpr uint32_t BatterySendIntervalMs = Sender::BatterySendIntervalMs;
    constexpr uint32_t SupportedPidRefreshIntervalMs = Sender::SupportedPidRefreshIntervalMs;
    constexpr uint32_t DtcQueryIntervalMs = Sender::DtcQueryIntervalMs;
    constexpr bool EnableTelemetrySimulation = Feature::EnableSenderTelemetrySimulation;
    constexpr uint32_t SimulationIntervalMs = Sender::SimulationIntervalMs;
    constexpr twai_mode_t TWAI_OPERATION_MODE = Sender::TwaiOperationMode;

    constexpr int LedPin1 = Sender::LedPin1;
    constexpr int LedPin2 = Sender::LedPin2;
    constexpr int ButtonPin = Sender::ButtonPin;
    constexpr int SHIELD_CAN_RX = Sender::ShieldCanRx;
    constexpr int SHIELD_CAN_TX = Sender::ShieldCanTx;
    constexpr int VoltageDividerPin = Sender::VoltageDividerPin;

    constexpr uint32_t StartStopDelayMs = Sender::StartStopDelayMs;
    constexpr int CpuFrequency = Sender::CpuFrequency;
    constexpr float VoltageCalcFactor = Sender::VoltageCalcFactor;
    constexpr float VoltageChangeThreshold = Sender::VoltageChangeThreshold;

    constexpr const char* WifiSsid = Network::WifiSsid;
    constexpr const char* WifiPassword = Network::WifiPassword;
    constexpr bool WifiEnable = Network::WifiEnable;

    constexpr const char* BluetoothName = Network::BluetoothName;
    constexpr bool BluetoothEnable = Feature::EnableBluetooth;

    constexpr const char* OtaHostname = Network::SenderOtaHostname;
    constexpr const char* OtaPassword = Network::SenderOtaPassword;
    constexpr bool OtaEnable = Feature::EnableSenderOta;
    constexpr uint32_t UpdateIntervalMs = 10000;

    constexpr const char* UpdateCheckUrl =
        "https://raw.githubusercontent.com/user/repo/main/firmware.bin";
    constexpr const char* FirmwareBinUrl =
        "https://raw.githubusercontent.com/user/repo/main/firmware.bin";

    constexpr bool UseEspNowEncryption = Network::UseEspNowEncryption;
    constexpr uint8_t EspNowAesKey[16] = {
        0x3A, 0x7F, 0xC2, 0x91, 0x18, 0x5D, 0xE0, 0xB3,
        0x4C, 0x22, 0xA1, 0x6E, 0xD4, 0x0F, 0x97, 0x8B
    };
    constexpr uint8_t EspNowPeerMac[6] = { 0xF0, 0xF5, 0xBD, 0x43, 0x29, 0x20 };

    // =========================================================================
    // Gemeinsame Datenstrukturen / Laufzeitwerte
    // =========================================================================
    struct VoltageMeasurement {
        int adcRaw = 0;
        float voltage = 0.0f;
    };

    inline uint32_t currentMillis = 0;
    inline uint32_t lastCanMsgTimestamp = 0;
    inline uint32_t lastObdRequestTime = 0;
    inline uint32_t lastCarRunningTime = 0;
    inline uint32_t lastBatterySendTime = 0;
    inline uint32_t ledTestStartTime = 0;
    inline float lastSpeed = 0.0f;
    inline float lastFuelRate = 0.0f;
    inline float consumption = 0.0f;
    inline float consumptionSum = 0.0f;
    inline int consumptionCount = 0;

    inline bool carIsRunning = false;
    inline bool ledTestActive = false;
    inline uint8_t ledTestStep = 0;

}
