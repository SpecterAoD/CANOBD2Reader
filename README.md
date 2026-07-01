# CANOBD2Reader

Ein PlatformIO-Projekt mit zwei getrennten Firmware-Zielen:

- `env:sender`: ESP32 DevKit V1 als CAN-/OBD2-Reader und ESP-NOW-Sender
- `env:display`: LilyGO T-Display S3 als ESP-NOW-Empfänger und Dashboard-Anzeige

## Projektstruktur

```text
CANOBD2Reader/
├── platformio.ini
├── include/
│   ├── common_config.h
│   ├── PIDs.h
│   ├── PID_Converter.h
│   ├── TelemetryProtocol.h
│   └── SimulationData.h
├── lib/
│   └── common/
│       ├── protocol.h
│       ├── protocol.cpp
│       ├── shared_types.h
│       ├── simulation_data.h
│       ├── CANDecoder.cpp
│       ├── PID_Converter.cpp
│       └── README.md
├── src/
│   ├── sender/
│   │   ├── main.cpp
│   │   ├── CANHandler.cpp
│   │   ├── OBDHandler.cpp
│   │   ├── OTAHandler.cpp
│   │   ├── NetworkManager.cpp
│   │   └── WebConsoleHandler.cpp
│   └── display/
│       └── main.cpp
└── docs/
    ├── telemetry.md
    └── archive/
```

Die ehemaligen Arduino-IDE-Sketches wurden migriert:

- `ino/CAN_OBD2_Gateway.ino` → `src/sender/main.cpp`
- `ino/Anzeige_LilyGoTDisplayS3.ino` → `src/display/main.cpp`

Die vorhandenen Sender-Module (`CANHandler`, `OBDHandler`, `OTAHandler`,
`NetworkManager`, `WebConsoleHandler`) sind wieder Bestandteil des aktiven
Sender-Builds. `src/sender/main.cpp` koordiniert Initialisierung und Loop,
während CAN-, OBD2- und OTA-Logik in den jeweiligen `.cpp/.h`-Modulen liegt.

Ältere, nicht mehr aktive Referenzdateien liegen zusätzlich unter `docs/archive/`.

## Build

Beide Targets:

```bash
platformio run
```

Nur Sender:

```bash
platformio run -e sender
```

Nur Display:

```bash
platformio run -e display
```

Die Firmware-Version wird beim Build aus `VERSION.txt` gelesen und per
`CANOBD2_FIRMWARE_VERSION` in beide Firmwares eingebettet. Lokal kann die Version
bei Bedarf überschrieben werden:

```bash
CANOBD2_FIRMWARE_VERSION=V1.2.3 platformio run -e sender -e display
```

## Flashen

Sender per USB:

```bash
platformio run -e sender -t upload
```

Display per USB:

```bash
platformio run -e display -t upload
```

Beide Geraete in einem Durchgang (mit automatischem Display-Retry):

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\upload_both.ps1
```

Das Skript baut beide Environments, flasht zuerst den Sender und versucht dann
das Display mehrfach. Bei Display-Fehlern erscheint ein Hinweis fuer den
Bootloader-Modus (BOOT halten, RST kurz tippen, BOOT nach 1-2 Sekunden loslassen).

Optional ohne Build:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\upload_both.ps1 -SkipBuild
```

Optional den initialen Bootloader-Prompt deaktivieren:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\upload_both.ps1 -NoDisplayBootPrompt
```

Serieller Monitor:

```bash
platformio device monitor -e sender
platformio device monitor -e display
```

## OTA

`platformio.ini` verwendet für beide Environments:

```ini
board_build.partitions = partitions/ota_4mb.csv
```

Das erhält ein OTA-kompatibles Layout mit zwei App-Partitionen für typische 4-MB-ESP32-Boards. OTA wird nicht im Code deaktiviert.

OTA-Upload kann lokal je Environment aktiviert werden, zum Beispiel:

```bash
platformio run -e sender -t upload --upload-port 192.168.0.50
platformio run -e display -t upload --upload-port 192.168.0.51
```

Falls nötig, in `platformio.ini` ergänzen:

```ini
upload_protocol = espota
upload_port = <ip-adresse>
```

Runtime-Verhalten:

- Sender startet OTA über `OTAHandler`. Wenn keine WLAN-Verbindung vorhanden ist, wird der zentrale Sender-SoftAP `Config::Network::SenderWebSsid` gestartet.
- Display besitzt ein eigenes `DisplayOta`-Modul und startet bei aktiviertem Flag den zentralen Display-SoftAP `Config::Network::DisplayWebSsid`.
- Beide Geräte verwenden `WIFI_AP_STA`, damit OTA/WebConsole und ESP-NOW parallel grundsätzlich möglich bleiben.
- Der Sender startet im Auto-Betrieb standardmäßig automatisch. `SenderConfig::RequireWebStart` ist `false`; das Webinterface ist damit nicht mehr erforderlich, um CAN/OBD und ESP-NOW zu starten.
- Der Sender sendet mindestens alle `SenderConfig::HeartbeatIntervalMs` ein Heartbeat-/Statuspaket per ESP-NOW, auch wenn noch keine OBD-Antwort oder kein CAN-Frame vorliegt.

### Sender-Start und Statuslogik

Beim Einschalten erwartet der serielle Monitor u. a.:

```text
[SENDER] Auto start enabled
[ESP-NOW] Heartbeat sent seq=...
```

Die Statusanzeigen sind bewusst getrennt:

- ESP-NOW: grün, sobald gültige Telemetrie- oder Heartbeat-Pakete empfangen werden.
- CAN: zeigt den CAN-/TWAI-Zustand des Senders; kann warnen oder rot sein, obwohl ESP-NOW grün ist.
- OBD: zeigt, ob echte ECU-Antworten eintreffen; fehlende OBD-Antworten machen nicht automatisch ESP-NOW rot.
- Simulation: zeigt nur, ob Simulationsdaten aktiv sind.

Fehlersuche im Auto:

1. Wenn ESP-NOW rot bleibt: MAC-Adressen, ESP-NOW-Kanal und AES-Key in `include/secrets.h` prüfen.
2. Wenn ESP-NOW grün, aber OBD rot/orange ist: CAN-Verkabelung, OBD-Pins, Zündung und Fahrzeugunterstützung prüfen.
3. Wenn CAN rot ist: TWAI-Initialisierung, Transceiver, CAN-H/L und Baudrate prüfen.
4. Wenn Werte grau sind: Pakete kommen, aber einzelne Messwerte sind älter als `DisplayConfigValues::ValueTimeoutMs`.

### OTA-Version und Status pruefen

`partitions/ota_4mb.csv` stellt `ota_0` und `ota_1` mit je `0x1D0000`
Bytes bereit. Beide Weboberflaechen zeigen Firmware-Version, Target,
Protokollversion, Build-Zeit, freien OTA-Speicher, Sketch-Groesse,
Flash-Groesse, IP und Laufzeit. Dieselben Werte sind unter `/status` als JSON
verfuegbar.

Nach einem Web-OTA kann die installierte Version direkt geprueft werden:

- Sender: `http://192.168.4.1/status` im Sender-WLAN aus `include/secrets.h`
- Display: `http://192.168.4.1/status` im Display-WLAN aus `include/secrets.h`

Wichtige JSON-Felder:

```json
{
  "firmware": "V1.0.0",
  "target": "sender",
  "protocol": 2,
  "freeSketchSpace": 1900544,
  "sketchSize": 854745,
  "flashSize": 4194304
}
```

OTA-Fehler erscheinen direkt in der Weboberflaeche und im `/status` Feld
`otaStatus`. Typische Ursachen sind eine falsche `.bin` fuer das Zielgeraet,
zu wenig freier OTA-Speicher oder ein abgebrochener Upload.

## Debug-/Feature-Flags

Die zentralen Build-Flags liegen in `platformio.ini` und `include/common_config.h`:

```ini
-DCANOBD2_ENABLE_BLUETOOTH=0
-DCANOBD2_ENABLE_SENDER_WEBCONSOLE=1
-DCANOBD2_ENABLE_DISPLAY_OTA=1
```

Bluetooth ist standardmäßig deaktiviert und wird bei `0` nicht mitkompiliert.

Die Sender-WebConsole ist über den in `include/secrets.h` konfigurierten SoftAP erreichbar, wenn `CANOBD2_ENABLE_SENDER_WEBCONSOLE=1` und `Config::EnableWebConsole` aktiv sind.

## Zentrale Konfiguration

Alle gemeinsamen Laufzeitwerte liegen in `include/Config.h`. Die wichtigsten Bereiche:

- `Config::Network`: SSIDs, Passwörter, Webserver-Port, OTA-Hostnamen, ESP-NOW-Kanal, ESP-NOW-AES-Key und feste Peer-MAC-Adressen.
- `Config::Display`: Display-Power-Pin, Backlight-Pin, Button-Pin, Rotation, Seitenanzahl und UI-Timeouts.
- `Config::Sender`: CAN-/OBD2-Pins, Polling-/Timeout-Werte und Power-Messung.
- `Config::Feature`: Bluetooth, WebConsole, Display-Web-OTA und Simulation.

Aktuelle WLAN-/OTA-Werte werden nicht mehr fest im Repository gepflegt. Für echte Geräte `include/secrets.example.h` nach `include/secrets.h` kopieren und dort setzen:

| Ziel | SSID | Passwort | Zweck |
| --- | --- | --- | --- |
| Sender | `Secrets::SenderWebSsid` | `Secrets::SenderWebPassword` | WebConsole + Web-OTA |
| Display | `Secrets::DisplayWebSsid` | `Secrets::DisplayWebPassword` | Display-Web-OTA |

ESP-NOW nutzt zentral `Config::Network::EspNowChannel`. Wenn Web-OTA/AP und ESP-NOW parallel laufen, müssen beide Geräte auf demselben Kanal bleiben.

### Hinweis: Display bleibt nach Upload schwarz

Beim LilyGO T-Display S3 wird die Displayversorgung auf vielen Revisionen über GPIO 15 eingeschaltet. Dieser Wert ist jetzt zentral gesetzt:

```cpp
Config::Display::PowerPin = 15
Config::Display::BacklightPin = 38
```

Falls eine andere Board-Revision verwendet wird und das Display trotzdem dunkel bleibt, zuerst prüfen:

1. Wurde wirklich `env:display` auf das LilyGO T-Display S3 geflasht?
2. Passt der Power-Pin zur Board-Revision? Falls nicht, `PowerPin` in `include/Config.h` anpassen oder auf `-1` setzen.
3. Erscheint im seriellen Monitor ein Boot-Log?
4. Ist die Display-Firmware unter `.pio/build/display/firmware.bin` verwendet worden, nicht die Sender-Firmware?

### Sender-WebConsole

Die WebConsole ist für iPhone-Breite optimiert und wird ohne horizontales
Verschieben bedient. Nach dem Flashen:

1. Mit dem in `include/secrets.h` gesetzten Sender-WLAN verbinden (`Secrets::SenderWebSsid`).
2. Das dort gesetzte Passwort verwenden (`Secrets::SenderWebPassword`).
3. Im Browser `http://192.168.4.1/` öffnen.

Die Oberfläche hat vier Seiten, die über Buttons umgeschaltet werden:

- Status: CAN, OBD2, Batteriespannung, Telemetrie-Zähler, letztes Paket
- Diag: PID-Support, letzter CAN-Zeitstempel, DTCs, Fehlerstatus
- Log: Logger-/Systemmeldungen
- OTA: Firmwaredatei `firmware.bin` direkt über die Webseite hochladen

Standardmäßig wartet der Sender nicht mehr auf die WebConsole. Nur wenn
`SenderConfig::RequireWebStart = true` gesetzt wird, dient der Button
`Sender starten` als manuelle Freigabe. OTA, WebConsole und Heartbeat bleiben
auch in diesem manuellen Modus aktiv.

### Web-OTA vom iPhone

Beide Geräte können ihre Firmware direkt über Safari/Dateien auf dem iPhone
hochladen. Wichtig: Immer die zum Gerät passende Datei verwenden.

Sender:

1. iPhone mit dem Sender-WLAN aus `include/secrets.h` verbinden.
2. Das dort gesetzte Sender-Web-Passwort eingeben.
3. `http://192.168.4.1/` öffnen.
4. Seite `OTA` wählen.
5. Sender-Firmware aus `.pio/build/sender/firmware.bin` hochladen.

Display:

1. iPhone mit WLAN `CANOBD2_Display_OTA` verbinden.
2. Passwort `Update123` eingeben.
3. `http://192.168.4.1/` öffnen.
4. Display-Firmware aus `.pio/build/display/firmware.bin` hochladen.

Nach erfolgreichem Upload startet das jeweilige Gerät automatisch neu.

## Automatischer GitHub-Build

Bei jedem Push auf `main` oder `master` baut GitHub Actions automatisch:

- `env:sender`
- `env:display`

Dabei wird `VERSION.txt` automatisch um eine Patch-Version erhöht, z. B.
`V1.0.0` → `V1.0.1`. Diese Version wird auch in die Firmware selbst eingebettet
und für die Dateinamen verwendet.

Die erzeugten Dateien liegen danach als GitHub-Actions-Artefakt und zusätzlich
als GitHub Release vor:

- `CAN_OBD2_sender_<VERSION>.bin`
- `CAN_OBD2_display_<VERSION>.bin`
- `CAN_OBD2_sender_<VERSION>.elf`
- `CAN_OBD2_display_<VERSION>.elf`
- `firmware_manifest.json`

Für Web-OTA vom iPhone werden normalerweise nur die `.bin`-Dateien benötigt.

## Telemetrie-Protokoll

Sender und Display verwenden ein gemeinsames CRC-geschütztes ESP-NOW-Binärpaket:

- `lib/telemetry/TelemetryPacket.h`
- `lib/telemetry/TelemetryCodec.h`
- `lib/telemetry/TelemetrySequence.h`

Der Payload bleibt als lesbare CSV-Zeile aufgebaut:

```text
TYPE,KEY,NAME,VALUE,UNIT,STATUS,SEQ
```

Beispiele:

```text
OBD,0C,RPM,1850.00,rpm,OK,42
OBD,05,CoolantTemp,89.00,°C,OK,43
BATTERY,VOLTAGE,BatteryVoltage,12.64,V,OK,44
STATUS,CAN,CAN,ACTIVE,,OK,45
```

Die Display-Seite validiert Paketgröße, Magic Number, Protokollversion und CRC,
bevor Werte in die UI übernommen werden. Die Sequenznummer im Paketheader ist
maßgeblich für Paketverlust-Erkennung; die CSV-Sequenz dient nur der Diagnose.

Details: `docs/telemetry.md`

## Architektur und Tests

Zusätzliche Dokumentation:

- `docs/architecture.md`: Modulaufteilung, Datenfluss, OTA und Legacy-Ablage
- `docs/isotp.md`: ISO-TP-Reassembly, Flow-Control und OBD-II-Integration
- `docs/telemetry.md`: Paketformat und Display-Felder

Native Unit Tests:

```bash
platformio test -e native
```

Unter Windows benötigt `env:native` einen installierten C/C++-Compiler
(`gcc`/`g++`, z. B. über MSYS2/MinGW). GitHub Actions führt diese Tests auf
Linux automatisch aus.

Aktuell abgedeckt:

- CRC16
- PID-Dekoder
- MAP/BARO-Dekoder und zentrale Ladedruckberechnung
- ISO-TP Single/First/Consecutive Frames und Fehlerfälle
- TelemetryCodec
- zentrale Konfiguration
- Display-Severity und Warn-/Störfarben
- Display-Simulationswerte fuer OK/Warn/Critical/Timeout inklusive RPM und Boost

## Display-Seiten

1. Hauptanzeige: Geschwindigkeit, Drehzahl, Kühlmitteltemperatur, Batteriespannung
2. Motorwerte: Öltemperatur, Kühlmitteltemperatur, Motorlast, Ansauglufttemperatur
3. Verbrauch: Durchschnittsverbrauch, Kraftstoffrate, Geschwindigkeit, Drosselklappe
4. Zusatzwerte: MAF, Tankfüllstand, Motorlaufzeit, Umgebungstemperatur
5. CAN-Rohdaten: letzter CAN-Frame, Frame-Zähler, einfache OBD/CAN-Hinweise
6. Diagnose: ESP-NOW, CAN, OBD, Heartbeat, Datenqualität und Paketverlust
7. Fehlercodes: DTC-Status und aktive Fehlercodes
8. Drehzahl-Grafik: grosser RPM-Wert, Balken 0 bis `DisplayConfigValues::RpmMax`, Warn-/Kritisch-Marken und Max-RPM seit Start
9. Ladedruck: fertiger `BoostPressureBar` in bar plus MAP und BARO als Zusatzwerte

### Ladedruck-Berechnung

MAP ist Absolutdruck und wird nicht direkt als Ladedruck angezeigt. Der Sender
berechnet den relativen Ladedruck zentral und uebertraegt ihn als eigene
Telemetrie `BoostPressureBar`:

```cpp
boostPressureKpa = manifoldAbsolutePressureKpa - barometricPressureKpa;
boostPressureBar = boostPressureKpa / 100.0f;
```

Verwendete PIDs:

- `0x0B` = Intake Manifold Absolute Pressure / MAP in kPa
- `0x33` = Barometric Pressure / BARO in kPa

Falls PID `0x33` vom Fahrzeug nicht unterstuetzt wird, nutzt der Sender
`SenderConfig::DefaultBarometricPressureKpa` (`101.3 kPa`). Das Display rechnet
den Boost nicht nach, sondern zeigt ausschliesslich den vom Sender gesendeten
Wert an. Die Boost-Farben sind zentral konfiguriert:

- `< 0.8 bar`: OK/gruen
- `0.8 bis 1.2 bar`: Warnung/orange
- `> 1.2 bar`: kritisch/rot

## Runtime-Simulation und Web-Schalter

Die neue Simulation ist standardmaessig aus und wird nicht in NVS, EEPROM oder
Flash gespeichert. Nach jedem Neustart gilt wieder:

```cpp
SimulationConfig::EnableSimulationByDefault == false
```

Sender und Display besitzen in der Weboberflaeche Buttons fuer Simulation und
Neustart. Die Steuerung wirkt nur im RAM der aktuellen Laufzeit.

Neue API-Endpunkte auf beiden Geraeten:

| Methode | Pfad | Zweck |
| --- | --- | --- |
| `GET` | `/api/simulation` | Runtime-Status und Szenario lesen |
| `POST` | `/api/simulation/on` | Simulation einschalten |
| `POST` | `/api/simulation/off` | Simulation ausschalten |
| `POST` | `/api/simulation/toggle` | Simulation umschalten |
| `GET` | `/api/simulation/scenario` | aktuelles Szenario lesen |
| `POST` | `/api/simulation/scenario?scenario=<Name>` | Szenario setzen |
| `POST` | `/api/restart` | Antwort senden und ESP neu starten |

Unterstuetzte ISO-TP-/OBD-Szenarien:

- `NormalSingleFrame`
- `NormalMultiFrameVin`
- `NormalMultiFrameDtc`
- `FlowControlRequired`
- `TimeoutAfterFirstFrame`
- `SequenceError`
- `BufferOverflow`
- `MultipleEcusResponse`
- `NegativeResponse`
- `DisplayNormalValues`
- `DisplayWarningValues`
- `DisplayCriticalValues`
- `DisplayTimeoutValues`
- `DisplayMixedValues`

Die Display-Farben werden zentral ueber eine Severity-Logik abgeleitet:

- OK: gruen
- Warnung: orange
- Stoerung/Kritisch: rot
- Timeout/unbekannt: grau

Die Szenarien `DisplayNormalValues`, `DisplayWarningValues`,
`DisplayCriticalValues`, `DisplayTimeoutValues` und `DisplayMixedValues`
erzeugen gezielt Werte, mit denen alle Displayfarben ohne Fahrzeug getestet
werden koennen.

Sie enthalten auch RPM- und Ladedruckwerte:

- Normal: MAP 120 kPa, BARO 101 kPa, Boost 0.19 bar
- Warnung: MAP 190 kPa, BARO 101 kPa, Boost 0.89 bar
- Kritisch: MAP 250 kPa, BARO 101 kPa, Boost 1.49 bar
- Timeout: Werte werden bewusst als veraltet markiert und grau angezeigt

Die Display-Diagnoseseite zeigt zusaetzlich Firmware-Version, Sequenznummer,
Simulation aktiv/inaktiv und das aktive Simulationsszenario. Die Weboberflaechen
zeigen dieselben Informationen ueber `/status` und `/api/simulation`.

Display-Button:

- kurzer Druck: naechste Seite
- langer Druck: zur Main Page zurueck

Sender-Button:

- gedrueckt: beide LEDs als LED-Test einschalten
- losgelassen: normaler LED-Modus
- entprellt und ohne blockierende Delays

## Sicherheit und lokale Secrets

Sensible Werte liegen nicht mehr direkt in `Config.h`. Das Repository enthält
nur `include/secrets.example.h` mit Platzhaltern. Für echte Geräte:

1. `include/secrets.example.h` nach `include/secrets.h` kopieren.
2. WLAN-Passwörter, Web-Login, OTA-Passwort, ESP-NOW-Key und MAC-Adressen
   dort setzen.
3. `include/secrets.h` nicht committen; die Datei ist in `.gitignore`
   eingetragen.

Die Sender- und Display-Weboberflächen sind standardmäßig per Basic
Authentication geschützt (`SecurityConfig::EnableAuthentication = true`).
Geschützt sind insbesondere:

- Web-OTA
- Neustart-Endpunkte
- Simulation-Endpunkte
- Status-/Log-Webseiten

Skripte können alternativ zu Basic Auth den konfigurierten `Secrets::ApiToken`
nutzen:

- Header `X-API-Token`
- Header `Authorization: Bearer <token>`
- Query-Parameter `token`

Weitere Details stehen in `docs/security.md`.

## End-to-End Test Sender zu Display

Dieser Ablauf prueft die komplette Kette in einem Durchgang:

1. Build und Upload beider Geraete:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\upload_both.ps1
```

2. Serielle Monitore oeffnen:

```bash
platformio device monitor -e sender
platformio device monitor -e display
```

3. Erwartete Sender-Traceausgabe (ca. jede Sekunde):

```text
[sender-tx] seq=... ok=... fail=... sim=on
```

4. Erwartete Display-Traceausgabe (ca. jede Sekunde):

```text
[display-rx] text=... can=... crc=... badlen=... macdrop=... parse=... lastSeq=...
```

5. Erfolgskriterien:

- Sender: `ok` steigt kontinuierlich, `fail` bleibt 0 oder sehr niedrig.
- Display: `text` (und je nach Modus `can`) steigt kontinuierlich.
- Displayseiten zeigen Werte statt `--` auf den OBD- und Batterie-Feldern.
- `crc`, `badlen`, `parse` bleiben nahe 0.

6. Fehlerbild schnell einordnen:

- Sender `ok` steigt, Display `text` bleibt 0: Funkpfad/MAC/Kanal pruefen.
- Display `macdrop` steigt: Sender-MAC passt nicht zu `Config::Network::SenderAllowedMac`.
- Display `crc` steigt: AES-Key/Kanal oder Frame-Integritaet pruefen.
