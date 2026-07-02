#pragma once

#include <Arduino.h>
#include <driver/twai.h>

#include "BuildConfig.h"
#include "DisplayConfig.h"
#include "PIDs.h"
#include "ProjectConfig.h"
#include "SecurityConfig.h"
#include "SenderConfig.h"
#include "SimulationConfig.h"

namespace Config {

    // Config.h is the application-facing compatibility facade. New constants
    // should be added to ProjectConfig, BuildConfig, SenderConfig or
    // DisplayConfig first, then exposed here only when existing modules need
    // the historical Config::<Area>::<Name> access pattern.

    // =========================================================================
    // Projekt / Firmware
    // =========================================================================
    namespace Project {
        constexpr const char* FirmwareVersion = ProjectConfig::FirmwareVersion;
        constexpr uint8_t ProtocolVersion = ProjectConfig::ProtocolVersion;
        constexpr const char* TargetName = ProjectConfig::TargetName;
        constexpr int Baudrate = 115200;
    }

    // =========================================================================
    // Feature Flags
    // =========================================================================
    namespace Feature {
        constexpr bool EnableBluetooth = BuildConfig::BluetoothEnabled;
        constexpr bool EnableSenderWebConsole = BuildConfig::SenderWebConsoleEnabled;
        constexpr bool EnableDisplayWebOta = BuildConfig::DisplayOtaEnabled;
        constexpr bool EnableSenderOta = true;

        // Sender-Simulation: sendet ohne Fahrzeug/CAN-Transceiver Testwerte.
        constexpr bool EnableSenderTelemetrySimulation = SimulationConfig::EnableSimulationByDefault;

        // Display-Simulation: erzeugt lokal Testwerte fuer alle Display-Seiten.
        constexpr bool EnableDisplayInternalSimulation = SimulationConfig::EnableSimulationByDefault;
    }

    // =========================================================================
    // Netzwerk, Weboberflaechen, OTA und ESP-NOW
    // =========================================================================
    namespace Network {
        constexpr const char* BluetoothName = "ESP-BT";

        // WebConsole / OTA Access Point des Senders.
        constexpr const char* SenderWebSsid = Secrets::SenderWebSsid;
        constexpr const char* SenderWebPassword = Secrets::SenderWebPassword;
        constexpr const char* SenderOtaHostname = "CAN_OBD2_Gateway";
        constexpr const char* SenderOtaPassword = Secrets::SenderOtaPassword;

        // Web-OTA Access Point des Displays.
        constexpr const char* DisplayWebSsid = Secrets::DisplayWebSsid;
        constexpr const char* DisplayWebPassword = Secrets::DisplayWebPassword;
        constexpr const char* DisplayOtaHostname = "CANOBD2_Display";

        constexpr uint16_t WebServerPort = 80;
        constexpr size_t WebConsoleMaxLines = 50;
        constexpr bool RequireWebStart = SenderConfig::RequireWebStart;

        // Optionales stationaeres WLAN fuer spaetere Erweiterungen.
        constexpr const char* WifiSsid = Secrets::WifiSsid;
        constexpr const char* WifiPassword = Secrets::WifiPassword;
        constexpr bool WifiEnable = WifiSsid[0] != '\0';

        // ESP-NOW: Sender sendet an die Display-MAC; Display akzeptiert nur die
        // Sender-MAC. Beide Werte muessen zu den realen Geraeten passen.
        constexpr uint8_t EspNowChannel = ProjectConfig::EspNowChannel;
        constexpr bool UseEspNowEncryption = true;
        constexpr const uint8_t* EspNowAesKey = Secrets::EspNowAesKey;
        constexpr const uint8_t* DisplayPeerMac = Secrets::DisplayPeerMac;
        constexpr const uint8_t* SenderAllowedMac = Secrets::SenderAllowedMac;
    }

    namespace Security {
        constexpr bool EnableAuthentication = SecurityConfig::EnableAuthentication;
        constexpr const char* AuthenticationRealm = SecurityConfig::AuthenticationRealm;
        constexpr const char* WebUsername = SecurityConfig::WebUsername;
        constexpr const char* WebPassword = SecurityConfig::WebPassword;
        constexpr const char* ApiToken = SecurityConfig::ApiToken;
        constexpr bool RequireOtaAuthentication = SecurityConfig::RequireOtaAuthentication;
        constexpr bool RequireSimulationAuthentication = SecurityConfig::RequireSimulationAuthentication;
        constexpr bool RequireRestartAuthentication = SecurityConfig::RequireRestartAuthentication;
        constexpr bool RejectOtaWhenSketchSpaceUnknown = SecurityConfig::RejectOtaWhenSketchSpaceUnknown;
        constexpr uint32_t RestartDelayMs = SecurityConfig::RestartDelayMs;
    }

    // =========================================================================
    // Display-Hardware und UI-Timing
    // =========================================================================
    namespace Display {
        constexpr uint8_t Rotation = DisplayConfigValues::Rotation;

        // LilyGO T-Display S3: Pin 15 schaltet die Displayversorgung frei.
        // Wenn eine Board-Revision diesen Pin nicht nutzt, auf -1 setzen.
        constexpr int8_t PowerPin = DisplayConfigValues::PowerPin;
        constexpr uint8_t PowerOnLevel = HIGH;
        constexpr uint16_t PowerStabilizeMs = DisplayConfigValues::PowerStabilizeMs;

        constexpr uint8_t BacklightPin = DisplayConfigValues::BacklightPin;
        constexpr uint8_t BacklightOn = DisplayConfigValues::BacklightOn;
        constexpr uint8_t NextPageButtonPin = DisplayConfigValues::NextPageButtonPin;
        constexpr uint8_t PageCount = DisplayConfigValues::PageCount;
        constexpr uint8_t MainPageIndex = DisplayConfigValues::MainPageIndex;
        constexpr uint32_t LongPressMs = DisplayConfigValues::LongPressMs;
        constexpr bool EnableGraphPages = DisplayConfigValues::EnableGraphPages;

        constexpr uint32_t ScreenRefreshMs = DisplayConfigValues::RefreshMs;
        constexpr uint32_t ForceFullRenderMs = DisplayConfigValues::ForceFullRenderMs;
        constexpr bool UseSegmentValueRenderer = DisplayConfigValues::UseSegmentValueRenderer;
        constexpr bool EnableStartupValueOverlay = DisplayConfigValues::EnableStartupValueOverlay;
        constexpr uint32_t StartupValueOverlayMs = DisplayConfigValues::StartupValueOverlayMs;
        constexpr float SpeedSmoothingAlpha = DisplayConfigValues::SpeedSmoothingAlpha;
        constexpr float RpmSmoothingAlpha = DisplayConfigValues::RpmSmoothingAlpha;
        constexpr uint32_t ConnectionTimeoutMs = DisplayConfigValues::ConnectionTimeoutMs;
        constexpr uint32_t EspNowTimeoutMs = DisplayConfigValues::EspNowTimeoutMs;
        constexpr uint32_t ObdTimeoutMs = DisplayConfigValues::ObdTimeoutMs;
        constexpr uint32_t CanTimeoutMs = DisplayConfigValues::CanTimeoutMs;
        constexpr uint32_t ValueTimeoutMs = DisplayConfigValues::ValueTimeoutMs;
        constexpr uint32_t ButtonDebounceMs = DisplayConfigValues::ButtonDebounceMs;
        constexpr float CoolantWarnC = DisplayConfigValues::CoolantWarnC;
        constexpr float CoolantCriticalC = DisplayConfigValues::CoolantCriticalC;
        constexpr float OilWarnC = DisplayConfigValues::OilWarnC;
        constexpr float OilCriticalC = DisplayConfigValues::OilCriticalC;
        constexpr float VoltageWarnLow = DisplayConfigValues::VoltageWarnLow;
        constexpr float VoltageCriticalLow = DisplayConfigValues::VoltageCriticalLow;
        constexpr float VoltageWarnHigh = DisplayConfigValues::VoltageWarnHigh;
        constexpr float VoltageCriticalHigh = DisplayConfigValues::VoltageCriticalHigh;
        constexpr uint16_t RpmWarn = DisplayConfigValues::RpmWarn;
        constexpr uint16_t RpmCritical = DisplayConfigValues::RpmCritical;
        constexpr uint16_t RpmMin = DisplayConfigValues::RpmMin;
        constexpr uint16_t RpmMax = DisplayConfigValues::RpmMax;
        constexpr float BoostWarnBar = DisplayConfigValues::BoostWarnBar;
        constexpr float BoostCriticalBar = DisplayConfigValues::BoostCriticalBar;
    }

    // =========================================================================
    // Sender-Hardware, CAN/OBD2 und Power-Management
    // =========================================================================
    namespace Sender {
        constexpr bool EnableCAN = SenderConfig::EnableCAN;
        constexpr bool EnableOBD2 = SenderConfig::EnableOBD2;
        constexpr bool EnableUDS = SenderConfig::EnableUDS;
        constexpr bool SendRawData = SenderConfig::SendRawData;
        constexpr bool SendRawDataOnly = SenderConfig::SendRawDataOnly;

        constexpr int LedPin1 = SenderConfig::LedPin1;
        constexpr int LedPin2 = SenderConfig::LedPin2;
        constexpr int ButtonPin = SenderConfig::ButtonPin;
        constexpr int ShieldCanRx = SenderConfig::CanRxPin;
        constexpr int ShieldCanTx = SenderConfig::CanTxPin;
        constexpr int VoltageDividerPin = SenderConfig::VoltageDividerPin;

        constexpr int PollingRateMs = SenderConfig::PollingRateMs;
        constexpr int CanIdleTimeoutMs = SenderConfig::CanIdleTimeoutMs;
        constexpr int SleepPeriodSec = SenderConfig::SleepPeriodSec;
        constexpr uint32_t ObdIntervalMs = SenderConfig::ObdPollIntervalMs;
        constexpr uint32_t ObdResponseTimeoutMs = SenderConfig::ObdResponseTimeoutMs;
        constexpr uint32_t ObdTxTimeoutMs = SenderConfig::ObdTxTimeoutMs;
        constexpr uint32_t BatterySendIntervalMs = SenderConfig::BatterySendIntervalMs;
        constexpr uint32_t HeartbeatIntervalMs = SenderConfig::HeartbeatIntervalMs;
        constexpr uint32_t SimulationIntervalMs = SenderConfig::SimulationIntervalMs;
        constexpr uint32_t LedTestDebounceMs = SenderConfig::LedTestDebounceMs;
        constexpr uint32_t SupportedPidRefreshIntervalMs = SenderConfig::SupportedPidRefreshMs;
        constexpr uint32_t DtcQueryIntervalMs = SenderConfig::DtcQueryIntervalMs;
        constexpr uint32_t VinQueryIntervalMs = SenderConfig::VinQueryIntervalMs;
        constexpr uint32_t UdsQueryIntervalMs = SenderConfig::UdsQueryIntervalMs;
        constexpr uint32_t UdsResponseTimeoutMs = SenderConfig::UdsResponseTimeoutMs;
        constexpr bool EnablePhysicalObdFallback = SenderConfig::EnablePhysicalObdFallback;
        constexpr uint8_t FunctionalTimeoutsBeforePhysicalFallback = SenderConfig::FunctionalTimeoutsBeforePhysicalFallback;
        constexpr uint32_t StartStopDelayMs = SenderConfig::StartStopDelayMs;
        constexpr int CpuFrequency = SenderConfig::CpuFrequency;

        constexpr float VoltageCalcFactor = SenderConfig::VoltageCalcFactor;
        constexpr float VoltageChangeThreshold = SenderConfig::VoltageChangeThreshold;
        constexpr float DefaultBarometricPressureKpa = SenderConfig::DefaultBarometricPressureKpa;

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
        INTAKE_MANIFOLD_ABS_PRESSURE,
        VEHICLE_SPEED,
        INTAKE_AIR_TEMP,
        ABS_BAROMETRIC_PRESSURE,
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
    constexpr bool EnableUDS = Sender::EnableUDS;
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
    constexpr const uint8_t* EspNowAesKey = Network::EspNowAesKey;
    constexpr const uint8_t* EspNowPeerMac = Network::DisplayPeerMac;

    // =========================================================================
    // Gemeinsame Datenstrukturen / Laufzeitwerte
    // =========================================================================
    struct VoltageMeasurement {
        int adcRaw = 0;
        float voltage = 0.0f;
    };

}
