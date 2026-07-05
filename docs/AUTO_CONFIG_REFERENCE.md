# AUTO Config Reference

> AUTO-GENERATED FILE. Do not edit manually.
> Regenerate with the corresponding script in `scripts/`.

## Config headers

### `BuildConfig.h`

| Type | Name |
|---|---|
| `LogLevel` | `CompileTimeLogLevel` |
| `LogLevel` | `CompileTimeLogLevel` |
| `bool` | `BluetoothEnabled` |
| `bool` | `SenderWebConsoleEnabled` |
| `bool` | `DisplayOtaEnabled` |
| `bool` | `OtaCompatible` |

### `DisplayConfig.h`

| Type | Name |
|---|---|
| `uint8_t` | `Rotation` |
| `int8_t` | `PowerPin` |
| `uint8_t` | `PowerOnLevel` |
| `uint16_t` | `PowerStabilizeMs` |
| `uint8_t` | `BacklightPin` |
| `uint8_t` | `BacklightOn` |
| `uint8_t` | `BacklightSleep` |
| `uint8_t` | `NextPageButtonPin` |
| `uint8_t` | `PageCount` |
| `uint8_t` | `MainPageIndex` |
| `uint32_t` | `LongPressMs` |
| `bool` | `EnableGraphPages` |
| `uint32_t` | `ScreenRefreshMs` |
| `uint32_t` | `RefreshMs` |
| `uint32_t` | `ForceFullRenderMs` |
| `bool` | `UseSegmentValueRenderer` |
| `bool` | `EnableStartupValueOverlay` |
| `uint32_t` | `StartupValueOverlayMs` |
| `float` | `SpeedSmoothingAlpha` |
| `float` | `RpmSmoothingAlpha` |
| `uint32_t` | `ValueTimeoutMs` |
| `uint32_t` | `ConnectionTimeoutMs` |
| `uint32_t` | `EspNowTimeoutMs` |
| `uint32_t` | `ObdTimeoutMs` |
| `uint32_t` | `CanTimeoutMs` |
| `uint32_t` | `ButtonDebounceMs` |
| `std::size_t` | `TelemetryQueueLength` |
| `float` | `CoolantWarnC` |
| `float` | `CoolantCriticalC` |
| `float` | `OilWarnC` |
| `float` | `OilCriticalC` |
| `float` | `VoltageWarnLow` |
| `float` | `VoltageCriticalLow` |
| `float` | `VoltageWarnHigh` |
| `float` | `VoltageCriticalHigh` |
| `uint16_t` | `RpmWarn` |
| `uint16_t` | `RpmCritical` |
| `uint16_t` | `RpmMin` |
| `uint16_t` | `RpmMax` |
| `float` | `BoostWarnBar` |
| `float` | `BoostCriticalBar` |
| `uint16_t` | `Background` |
| `uint16_t` | `Panel` |
| `uint16_t` | `Text` |
| `uint16_t` | `Muted` |
| `uint16_t` | `Accent` |
| `uint16_t` | `Ok` |
| `uint16_t` | `Warn` |
| `uint16_t` | `Error` |

### `LoggingConfig.h`

| Type | Name |
|---|---|
| `bool` | `SerialEnabled` |
| `bool` | `GeneralDebugEnabled` |
| `bool` | `TwaiDebugEnabled` |
| `bool` | `PowerDebugEnabled` |
| `bool` | `CanDebugEnabled` |
| `bool` | `Obd2DebugEnabled` |
| `bool` | `TraceSenderTelemetry` |
| `bool` | `TraceDisplayTelemetry` |
| `uint32_t` | `TraceSummaryIntervalMs` |
| `bool` | `EnablePersistentDiagnosticLog` |
| `bool` | `PersistTelemetryPayloadsToDiagnosticLog` |
| `std::size_t` | `DiagnosticLogMaxBytes` |
| `char*` | `SenderDiagnosticLogPath` |
| `char*` | `SenderDiagnosticLogArchivePath` |
| `char*` | `DisplayDiagnosticLogPath` |
| `char*` | `DisplayDiagnosticLogArchivePath` |
| `uint32_t` | `TwaiStatusLogIntervalMs` |

### `NetworkConfig.h`

| Type | Name |
|---|---|
| `char*` | `BluetoothName` |
| `char*` | `SenderWebSsid` |
| `char*` | `SenderWebPassword` |
| `char*` | `SenderOtaHostname` |
| `char*` | `SenderOtaPassword` |
| `char*` | `DisplayWebSsid` |
| `char*` | `DisplayWebPassword` |
| `char*` | `DisplayOtaHostname` |
| `uint16_t` | `WebServerPort` |
| `std::size_t` | `WebConsoleMaxLines` |
| `bool` | `RequireWebStart` |
| `char*` | `WifiSsid` |
| `char*` | `WifiPassword` |
| `bool` | `EnableStationWifi` |
| `bool` | `WifiEnable` |
| `uint32_t` | `WifiConnectTimeoutMs` |
| `uint8_t` | `EspNowChannel` |
| `bool` | `UseEspNowEncryption` |
| `uint8_t*` | `EspNowAesKey` |
| `uint8_t*` | `DisplayPeerMac` |
| `uint8_t*` | `SenderAllowedMac` |
| `bool` | `BluetoothEnabled` |

### `ObdConfig.h`

| Type | Name |
|---|---|
| `std::size_t` | `ObdPidCount` |
| `std::size_t` | `PidCount` |
| `float` | `DefaultBarometricPressureKpa` |
| `uint32_t` | `SupportedPidRefreshIntervalMs` |
| `uint32_t` | `DtcQueryIntervalMs` |
| `uint32_t` | `VinQueryIntervalMs` |

### `PowerConfig.h`

| Type | Name |
|---|---|
| `bool` | `EnablePowerManager` |
| `bool` | `EnableStartStopDetection` |
| `uint32_t` | `ParkDetectionTimeoutMs` |
| `uint32_t` | `DisplaySleepAfterMs` |
| `uint32_t` | `WakeupDebounceMs` |
| `bool` | `DimDisplayWhileIdle` |
| `uint8_t` | `IdleBrightnessPercent` |
| `bool` | `EnableActivityScore` |
| `VehiclePowerProfile` | `ActiveProfile` |
| `float` | `EngineRunningRpm` |
| `float` | `MovingSpeedKph` |
| `float` | `ActiveEngineLoadPercent` |
| `float` | `ActiveThrottlePercent` |
| `float` | `BoardVoltageAwakeMin` |

### `ProjectConfig.h`

| Type | Name |
|---|---|
| `char*` | `FirmwareVersion` |
| `uint16_t` | `ProtocolMagic` |
| `uint16_t` | `TelemetryMagic` |
| `uint8_t` | `ProtocolVersion` |
| `char*` | `TargetName` |
| `int` | `Baudrate` |
| `uint8_t` | `EspNowChannel` |
| `uint16_t` | `CrcInitialValue` |
| `uint16_t` | `CrcPolynomial` |
| `std::size_t` | `TelemetryPayloadSize` |
| `std::size_t` | `MaxIsoTpPayload` |

### `SecurityConfig.h`

| Type | Name |
|---|---|
| `bool` | `EnableAuthentication` |
| `char*` | `AuthenticationRealm` |
| `char*` | `WebUsername` |
| `char*` | `WebPassword` |
| `char*` | `ApiToken` |
| `bool` | `RequireOtaAuthentication` |
| `bool` | `RequireSimulationAuthentication` |
| `bool` | `RequireRestartAuthentication` |
| `bool` | `BlockNetworkFeaturesOnPlaceholderSecrets` |
| `bool` | `RejectOtaWhenSketchSpaceUnknown` |
| `bool` | `RequireOtaTargetInFilename` |
| `uint32_t` | `RestartDelayMs` |

### `SenderConfig.h`

| Type | Name |
|---|---|
| `bool` | `EnableCAN` |
| `bool` | `EnableOBD2` |
| `bool` | `EnableUDS` |
| `bool` | `EnableSenderOta` |
| `bool` | `SendRawData` |
| `bool` | `SendRawDataOnly` |
| `bool` | `RequireWebStart` |
| `int` | `LedPin1` |
| `int` | `LedPin2` |
| `int` | `ButtonPin` |
| `int` | `CanRxPin` |
| `int` | `CanTxPin` |
| `int` | `VoltageDividerPin` |
| `uint32_t` | `CanBitrate` |
| `int` | `PollingRateMs` |
| `int` | `CanIdleTimeoutMs` |
| `uint32_t` | `ObdPollIntervalMs` |
| `std::size_t` | `MaxObdPidsPerTick` |
| `uint32_t` | `ObdResponseTimeoutMs` |
| `uint32_t` | `ObdTxTimeoutMs` |
| `uint32_t` | `BatterySendIntervalMs` |
| `uint32_t` | `HeartbeatIntervalMs` |
| `uint32_t` | `SimulationIntervalMs` |
| `uint32_t` | `LedTestDebounceMs` |
| `uint32_t` | `SupportedPidRefreshMs` |
| `uint32_t` | `DtcQueryIntervalMs` |
| `uint32_t` | `VinQueryIntervalMs` |
| `uint32_t` | `UdsQueryIntervalMs` |
| `uint32_t` | `UdsResponseTimeoutMs` |
| `uint32_t` | `UdsResponsePendingTimeoutMs` |
| `bool` | `EnablePhysicalObdFallback` |
| `uint8_t` | `FunctionalTimeoutsBeforePhysicalFallback` |
| `int` | `CpuFrequency` |
| `bool` | `EnablePersistentDiagnosticLog` |
| `bool` | `PersistTelemetryPayloadsToDiagnosticLog` |
| `std::size_t` | `DiagnosticLogMaxBytes` |
| `char*` | `DiagnosticLogPath` |
| `char*` | `DiagnosticLogArchivePath` |
| `uint32_t` | `TwaiStatusLogIntervalMs` |
| `std::size_t` | `MaxCanFramesPerTick` |
| `float` | `VoltageCalcFactor` |
| `float` | `DefaultBarometricPressureKpa` |
| `std::size_t` | `CanRxQueueLength` |
| `std::size_t` | `CanTxQueueLength` |
| `uint8_t` | `IsoTpBlockSize` |
| `uint8_t` | `IsoTpStMinMs` |
| `uint32_t` | `IsoTpConsecutiveFrameTimeoutMs` |

### `SimulationConfig.h`

| Type | Name |
|---|---|
| `bool` | `EnableSimulationByDefault` |
| `Simulation::Scenario` | `DefaultScenario` |

### `UdsConfig.h`

| Type | Name |
|---|---|
| `bool` | `EnableUdsWriteServices` |
| `bool` | `EnableSecurityAccess` |
| `bool` | `EnableCodingOrAdaptation` |
| `bool` | `EnableExtendedDiagnosticSessionForCapabilityScan` |
| `uint8_t` | `TesterPresentSubFunction` |
| `uint8_t` | `TesterPresentSuppressPositiveResponseSubFunction` |
| `std::size_t` | `EcuTargetCount` |
| `std::size_t` | `DidCandidateCount` |
| `EcuTarget* findTargetByRequestId(uint32_t requestId) {
    for (std::size_t` | `index` |
| `uint32_t responseIdForRequestId(uint32_t requestId) {
    const EcuTarget*` | `target` |
| `std::size_t DefaultScanTargetCount() {
    std::size_t` | `count` |
| `std::size_t DefaultDidScanCount() {
    std::size_t` | `count` |

### `UpdateConfig.h`

| Type | Name |
|---|---|
| `bool` | `EnableGithubUpdates` |
| `bool` | `EnableAutoUpdate` |
| `bool` | `AutoInstallUpdates` |
| `bool` | `CheckOnWifiConnect` |
| `bool` | `CheckOnBoot` |
| `bool` | `AllowPrerelease` |
| `bool` | `AllowDevelopment` |
| `FirmwareUpdate::UpdateChannel` | `DefaultChannel` |
| `bool` | `AllowRollback` |
| `bool` | `RequireConfirmationForRollback` |
| `bool` | `RequireTlsCertificateValidation` |
| `bool` | `AllowInsecureTlsForDevelopment` |
| `bool` | `BlockInstallWhileVehicleRunning` |
| `uint32_t` | `ManifestTimeoutMs` |
| `uint32_t` | `DownloadTimeoutMs` |
| `uint32_t` | `AutoCheckMinIntervalMs` |
| `uint32_t` | `RestartAfterUpdateMs` |
| `char*` | `ManifestUrl` |
| `char*` | `GitHubRootCa` |
