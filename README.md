# CANOBD2Reader

Stand: `V2.0.0.b6`

CANOBD2Reader ist ein PlatformIO-Projekt mit zwei getrennten Firmwares:

- `env:sender`: ESP32 DevKit V1 mit CAN-/OBD2-/UDS-Anbindung, ESP-NOW-Sender, WebConsole und OTA.
- `env:display`: LilyGO T-Display S3 mit ESP-NOW-Empfang, Dashboard, Diagnoseansichten, Web-OTA und GitHub-Update-Seite.

Das Projekt ist auf read-only Fahrzeugdiagnose ausgelegt. OBD-II Livewerte sind die primäre Datenquelle; UDS und CAN-Sniffer dienen der gezielten Diagnose und Discovery, nicht zum Verändern von Steuergeräten.

## Inhaltsverzeichnis

- [Aktueller Ist-Stand](#aktueller-ist-stand)
- [Repository-Struktur](#repository-struktur)
- [Build und Tests](#build-und-tests)
- [Firmware-Version](#firmware-version)
- [Konfiguration und Secrets](#konfiguration-und-secrets)
- [Sender](#sender)
- [Display](#display)
- [OTA und GitHub-Updates](#ota-und-github-updates)
- [Entwickler-Scripts](#entwickler-scripts)
- [GitHub Actions](#github-actions)
- [Dokumentation](#dokumentation)
- [Hardwaretests](#hardwaretests)

## Aktueller Ist-Stand

Umgesetzt:

- Ein PlatformIO-Projekt mit `sender`, `display` und `native`.
- Zentrale Konfiguration unter `include/config/`; `include/Config.h` existiert nicht mehr.
- Secrets über `include/secrets.example.h` und lokale `include/secrets.h`.
- Sender startet standardmäßig automatisch, ohne Webbutton.
- ESP-NOW Heartbeat unabhängig von OBD-Erfolg.
- OBD-Livewerte sind in Fast- und Slow-PIDs getrennt:
  - Fast: RPM, Geschwindigkeit, Spannung, Kühlmittel, Last.
  - Slow: MAP, MAF, BARO, Tank, Laufzeit, Öltemperatur, Kraftstoffrate usw.
- Display zeigt fehlende Werte nicht mehr grün-leer, sondern `WARTET`, `KEINE DATEN`, `KEIN NRC` oder graue Platzhalter.
- RAW-CAN-Telemetrie ist rate-limited aktiv und befüllt Diagnose-/CAN-HEX-Anzeigen.
- Separate RPM-Bogenseite wurde aus der Navigation entfernt; RPM bleibt auf der Hauptseite.
- Web-OTA validiert Target, Version, Schema und Protokoll.
- OTA-Metadaten nutzen keine endlos wachsende Kompatibilitäts-Versionsliste mehr.
- GitHub-Update-System mit Kanälen, Manifest, WLAN-/Hotspot-Zugangsdaten und manuellem Rollback.
- Native Tests für Config, Security, ISO-TP, OBD, UDS, Simulation, Runtime, CAN Router, Telemetrie und Update-Manifest.

## Repository-Struktur

```text
CANOBD2Reader/
├── .github/
│   ├── actions/firmware-build/
│   └── workflows/
├── docs/
├── include/
│   ├── common_config.h
│   ├── secrets.example.h
│   └── config/
├── lib/
│   ├── can_router/
│   ├── capabilities/
│   ├── common/
│   ├── display/
│   ├── isotp/
│   ├── logging/
│   ├── network/
│   ├── obd/
│   ├── power/
│   ├── runtime/
│   ├── simulation/
│   ├── status/
│   ├── telemetry/
│   ├── transport/
│   ├── uds/
│   ├── update/
│   └── web/
├── partitions/
├── scripts/
├── src/
│   ├── display/
│   └── sender/
├── test/
├── platformio.ini
└── VERSION.txt
```

Details stehen in [docs/02_Project_Structure.md](docs/02_Project_Structure.md).

## Build und Tests

```powershell
platformio run -e sender
platformio run -e display
platformio test -e native
```

Komfortcheck:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\dev_check.ps1
```

Sauberer Neubuild:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\clean_build.ps1
```

## Firmware-Version

Die Version steht in [VERSION.txt](VERSION.txt). Beim Build setzt `scripts/apply_firmware_version.py` daraus:

- `CANOBD2_FIRMWARE_VERSION`
- `CANOBD2_TARGET_NAME`
- `CANOBD2_PROTOCOL_VERSION`

Aktueller Stand:

```text
V2.0.0.b6
```

## Konfiguration und Secrets

Statische Konfiguration:

- `include/config/ProjectConfig.h`
- `include/config/SenderConfig.h`
- `include/config/DisplayConfig.h`
- `include/config/NetworkConfig.h`
- `include/config/SecurityConfig.h`
- `include/config/PowerConfig.h`
- `include/config/SimulationConfig.h`
- `include/config/LoggingConfig.h`
- `include/config/ObdConfig.h`
- `include/config/UdsConfig.h`
- `include/config/UpdateConfig.h`

Lokale Zugangsdaten:

```powershell
Copy-Item include\secrets.example.h include\secrets.h
```

`include/secrets.h` wird nicht committed. CI verwendet `secrets.example.h`.

Für die Inbetriebnahme ist aktuell erlaubt:

```cpp
SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets = false;
```

Das ist praktisch zum Testen, aber nicht für den Dauerbetrieb gedacht. Für echte Nutzung eigene Passwörter, API-Token, ESP-NOW-Key und MAC-Adressen setzen.

## Sender

Der Sender:

- initialisiert CAN/TWAI,
- fragt OBD-II live ab,
- führt optionale UDS-Read-only-Diagnose aus,
- sendet ESP-NOW Telemetrie,
- stellt WebConsole, Diagnose-Log, OTA und GitHub-Update bereit,
- sendet Heartbeats auch ohne OBD-Antwort.

Live-OBD ist priorisiert:

| Ring | PIDs | Ziel |
| --- | --- | --- |
| Fast | RPM, Speed, Spannung, Kühlmittel, Last | schnelle Fahrerwerte |
| Slow | MAP, IAT, BARO, MAF, Drossel, Tank, Runtime, Außentemp., Öl, FuelRate | Zusatzwerte |

Fast-PIDs laufen deutlich häufiger als Zusatzwerte, damit Hauptanzeigen nicht von optionalen Timeouts gebremst werden.

## Display

Aktuelle Seiten:

1. Hauptseite: Geschwindigkeit, Drehzahl, Kühlmittel, Bordspannung, ESP/CAN/OBD/DTC.
2. Motorseite: Öltemperatur, Kühlmittel, Last, Ansaugluft, MAP/MAF/Drossel.
3. Verbrauch/Trip: Momentan, Durchschnitt, Kraftstoffrate, Geschwindigkeit, Laufzeit, Tank.
4. Diagnose kompakt: ESP-NOW, CAN, OBD, Update-Alter, Heartbeat, Pakete, Firmware.
5. UDS/DTC: VIN, OBD-/UDS-DTC, NRC, Backoff.
6. CAN-Sniffer/RAW-CAN: Snifferstatus, Baseline, Frames, CAN-ID, letzter HEX-Frame, Hinweis.
7. Zusatzwerte: MAF, MAP, BARO, Außentemperatur, Steuergerätspannung, Runtime.
8. Power Management: VehicleState, ActivityScore, PowerCommand, DisplayState, Last CAN/OBD.

Fehlende Werte erscheinen grau und beschriftet, nicht mehr als grüne leere Box.

## OTA und GitHub-Updates

Beide Environments verwenden:

```ini
board_build.partitions = partitions/ota_4mb.csv
```

Firmware-BINs enthalten einen ASCII-Metadatenblock:

```text
CANOBD2_FW_METADATA_BEGIN;
target=sender|display;
version=V...;
schema=2;
protocol=2;
min_protocol=2;
max_protocol=2;
CANOBD2_FW_METADATA_END
```

Damit ist keine wachsende Liste alter Versionsnummern mehr nötig.

Web-OTA prüft:

- Dateiname passt zum Target,
- Metadatenblock vorhanden,
- Target passt,
- Version vorhanden,
- Schema vorhanden,
- Protokoll passt,
- Upload/Write/Update.end erfolgreich.

GitHub-Updates:

- `Stable`: normale Releases.
- `Beta`: Releases und Pre-Releases.
- `Development`: auch Test-/Development-Firmware.

Rollback ist manuell möglich, aber nie automatisch.

Hinweis zu WLAN/Hotspot und ESP-NOW:

Sender und Display nutzen ESP-NOW auf `NetworkConfig::EspNowChannel`.
Station-WLAN für GitHub-Updates teilt sich denselben ESP32-Funkchip. Wenn ein
Handy-Hotspot auf einem anderen Kanal liegt, kann ESP-NOW sonst scheinbar
zufällig ausfallen. Deshalb trennt `WifiStationManager` Station-WLAN
standardmäßig wieder, wenn der Hotspot-Kanal nicht zum ESP-NOW-Kanal passt.
Das Web-JSON `/api/wifi/status` zeigt dafür `stationChannel`,
`espNowChannel`, `channelMatchesEspNow` und `lastError`.

## Entwickler-Scripts

Alle Scripts liegen unter [scripts/](scripts/). Wichtige Gruppen:

| Script | Zweck |
| --- | --- |
| `doctor.ps1` | prüft lokale Toolchain: Git, Python, PlatformIO, esptool, pyserial. |
| `dev_check.ps1` | Sender/Display Build, Native Tests und PlatformIO Checks. |
| `clean_build.ps1` | entfernt `.pio` und baut frisch. |
| `list_ports.ps1` | zeigt COM-Ports. |
| `flash_sender.ps1` / `flash_display.ps1` | USB-Flash pro Gerät. |
| `ota_upload_sender.ps1` / `ota_upload_display.ps1` | OTA-Upload per IP. |
| `monitor_sender.ps1` / `monitor_display.ps1` | serieller Monitor. |
| `upload_both.ps1` | baut und flasht Sender + Display. |
| `backup_config.ps1` | sichert lokale Secrets/Konfig-Snapshots außerhalb des Repos. |
| `check_secrets.py` | erkennt versehentlich eingecheckte Secrets/Keys. |
| `generate_project_index.py` | erzeugt `docs/AUTO_PROJECT_INDEX.md`. |
| `generate_build_docs.py` | erzeugt `docs/AUTO_BUILD_FLAGS.md`. |
| `generate_config_reference.py` | erzeugt `docs/AUTO_CONFIG_REFERENCE.md`. |
| `generate_test_overview.py` | erzeugt `docs/AUTO_TEST_OVERVIEW.md`. |
| `export_firmware_info.py` | erzeugt Firmware-Artefakt-Übersicht mit SHA256. |
| `verify_docs.py` | prüft Markdown-Links. |
| `validate_manifest.py` | prüft Release-/Update-Manifeste. |
| `check_protocol_version.py` | prüft Protokollversion in Code/Build/Doku. |
| `make_release_notes.py` | erzeugt einfache Release Notes. |
| `prepare_beta.ps1` | bereitet lokale Beta-Versionen vor. |

## GitHub Actions

Wichtige Workflows:

- `build.yml`: normale CI für Push, PR und manuell.
- `test-build.yml`: manuelle Test-Firmware als Artifact, kein Release.
- `beta-release.yml`: sichtbare Beta als GitHub Pre-Release.
- `prerelease.yml`: manuelles PreRelease.
- `release.yml`: manuelles Stable Release.
- `docs.yml`: Doku-Check und Auto-Doku-Artefakt.
- `size-report.yml`: Firmware-Größenbericht.

Gemeinsame Build-Logik liegt in `.github/actions/firmware-build/action.yml`.

## Dokumentation

Startpunkt:

- [docs/README.md](docs/README.md)
- [docs/00_Master_Design_Document.md](docs/00_Master_Design_Document.md)
- [docs/01_Architecture.md](docs/01_Architecture.md)
- [docs/02_Project_Structure.md](docs/02_Project_Structure.md)
- [docs/26_Developer_Guide.md](docs/26_Developer_Guide.md)

Auto-generierte Referenzen:

- [docs/AUTO_PROJECT_INDEX.md](docs/AUTO_PROJECT_INDEX.md)
- [docs/AUTO_BUILD_FLAGS.md](docs/AUTO_BUILD_FLAGS.md)
- [docs/AUTO_CONFIG_REFERENCE.md](docs/AUTO_CONFIG_REFERENCE.md)
- [docs/AUTO_TEST_OVERVIEW.md](docs/AUTO_TEST_OVERVIEW.md)
- [docs/AUTO_FIRMWARE_ARTIFACTS.md](docs/AUTO_FIRMWARE_ARTIFACTS.md)

## Hardwaretests

Noch immer nur am Fahrzeug final prüfbar:

- echte OBD-Antwortzeiten pro PID,
- Fast-Live-Verhalten im Tiguan,
- CAN/UDS-Sniffer-Workflow,
- Web-Buttons für Capabilities im echten Betrieb,
- OTA von älterer installierter Firmware auf `V2.0.0.b6`,
- ESP-NOW-Reichweite und Kanalverhalten bei aktivem WLAN/Hotspot.
