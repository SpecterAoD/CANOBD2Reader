# CANOBD2Reader

Ein PlatformIO-Projekt mit zwei getrennten Firmware-Zielen:

- `env:sender`: ESP32 DevKit V1 als CAN-/OBD2-Reader und ESP-NOW-Sender
- `env:display`: LilyGO T-Display S3 als ESP-NOW-Empfänger und Dashboard-Anzeige

## Projektdokumentation

Die technische Projektdokumentation liegt im Ordner [`docs/`](docs/README.md). Sie ist als Single Source of Truth fuer Architektur, Konfiguration, Runtime, CAN/OBD/UDS, Telemetrie, Display, OTA, Simulation, Tests, Sicherheit und Roadmap gedacht.

Wichtige Einstiege:

- [`docs/00_Master_Design_Document.md`](docs/00_Master_Design_Document.md) - Projektvision, Zielarchitektur, Roadmap und Definition of Done
- [`docs/01_Architecture.md`](docs/01_Architecture.md) - Gesamtarchitektur und Datenfluss
- [`docs/02_Project_Structure.md`](docs/02_Project_Structure.md) - Ordner und Module
- [`docs/16_OTA.md`](docs/16_OTA.md) - OTA, Firmware-Metadaten und Release-Artefakte
- [`docs/25_Roadmap.md`](docs/25_Roadmap.md) - offene Baustellen und geplante Entwicklung
- [`docs/26_Developer_Guide.md`](docs/26_Developer_Guide.md) - Entwicklungsablauf, Build, Tests und Release

Bei Architektur- oder Verhaltensaenderungen sollen die passenden Dokumente im selben Commit aktualisiert werden.

## Projektstruktur

```text
CANOBD2Reader/
├── platformio.ini
├── include/
│   ├── common_config.h
│   ├── secrets.example.h
│   └── config/
│       ├── ProjectConfig.h
│       ├── SenderConfig.h
│       ├── DisplayConfig.h
│       ├── NetworkConfig.h
│       └── SecurityConfig.h
├── lib/
│   ├── telemetry/        # Paketformat, CRC, Sequenzen
│   ├── isotp/            # ISO-TP Request/Response
│   ├── obd/              # PID-/VIN-/DTC-Dekodierung
│   ├── uds/              # UDS-Client und Decoder
│   ├── runtime/          # Laufzeitzustand und Coordinator-Logik
│   ├── status/           # Zustands-/Heartbeat-Helfer
│   ├── web/              # Authentifizierung und OTA/Web-Helfer
│   ├── transport/        # ESP-NOW Telemetrie-Transport
│   ├── simulation/       # Native-/Runtime-Simulation
│   ├── display/          # Gemeinsame Display-Logik
│   ├── logging/          # Diagnose-Log
│   └── can_router/       # Vorbereitete CAN-Fan-out-Bausteine
├── src/
│   ├── sender/
│   │   ├── main.cpp
│   │   ├── SenderApp.cpp
│   │   ├── SenderObdScheduler.cpp
│   │   ├── SenderUdsScheduler.cpp
│   │   ├── SenderTelemetry.cpp
│   │   ├── SenderEspNow.cpp
│   │   └── WebConsoleHandler.cpp
│   └── display/
│       ├── main.cpp
│       ├── DisplayApp.cpp
│       ├── DisplayReceiver.cpp
│       ├── DisplayUi.cpp
│       └── DisplayOta.cpp
├── test/                 # Native Unity-Tests für Runtime, Security, Codec, ISO-TP, ...
└── docs/
    ├── architecture.md
    ├── security.md
    ├── telemetry.md
    └── archive/
```

Die ehemaligen Arduino-IDE-Sketches wurden migriert:

- `ino/CAN_OBD2_Gateway.ino` → `src/sender/main.cpp`
- `ino/Anzeige_LilyGoTDisplayS3.ino` → `src/display/main.cpp`

`SenderApp` und `DisplayApp` sind heute bewusst nur schmale Einstiegspunkte.
Die eigentliche Laufzeitlogik liegt in Modulen unter `src/*` sowie in gemeinsam
genutzten Bibliotheken unter `lib/*` (z. B. `runtime`, `web`, `telemetry`,
`isotp`, `obd`, `uds`).

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

Native Tests:

```bash
platformio test -e native
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

- Sender startet OTA über `OTAHandler`. Wenn keine WLAN-Verbindung vorhanden ist, wird der zentrale Sender-SoftAP `NetworkConfig::SenderWebSsid` gestartet.
- Display besitzt ein eigenes `DisplayOta`-Modul und startet bei aktiviertem Flag den zentralen Display-SoftAP `NetworkConfig::DisplayWebSsid`.
- Beide Geräte verwenden `WIFI_AP_STA`, damit OTA/WebConsole und ESP-NOW parallel grundsätzlich möglich bleiben.
- Der Sender startet im Auto-Betrieb standardmäßig automatisch. `SenderConfig::RequireWebStart` ist `false`; das Webinterface ist damit nicht mehr erforderlich, um CAN/OBD und ESP-NOW zu starten.
- Der Sender sendet mindestens alle `SenderConfig::HeartbeatIntervalMs` ein Heartbeat-/Statuspaket per ESP-NOW, auch wenn noch keine OBD-Antwort oder kein CAN-Frame vorliegt.
- Web-OTA prüft neben Authentifizierung und Dateinamen inzwischen auch eingebettete Firmware-Metadaten (Target/Firmware-Version) vor dem finalen Aktivieren des Uploads.

## CI / Releases

- Pull Requests und normale Branch-Builds kompilieren Sender + Display und führen die Native-Tests aus.
- Release-Erstellung ist vom Build getrennt und läuft nur für Tags (`firmware-*`, `v*`).
- Die Firmware-Artefakte enthalten zusätzlich ein `firmware_manifest.json` mit SHA-256-Hashes für Sender und Display.

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
4. Wenn Werte grau sind: Pakete kommen, aber einzelne Messwerte sind älter als `DisplayConfig::ValueTimeoutMs`.
5. Im Sender-Diagnose-Log prüfen:
   - `[ISOTP] Request sent ... pid=0x..`: OBD-Anfrage wurde wirklich auf CAN gesendet.
   - `[ISOTP] Response id=0x7E8...`: ECU hat geantwortet.
   - `[ISOTP] Timeout waiting for response`: Anfrage ging raus, aber keine ECU-Antwort kam zurück.
   - `[ISOTP] Negative response service=... nrc=...`: ECU hat die Anfrage aktiv abgelehnt.
   - `[OBD] Supported PIDs ...`: Supported-PID-Erkennung funktioniert.
   - Fehlen alle `[ISOTP] Request sent`-Zeilen, läuft der OBD-Polling-Pfad nicht.

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

Die Sender-WebConsole ist über den in `include/secrets.h` konfigurierten SoftAP erreichbar, wenn `CANOBD2_ENABLE_SENDER_WEBCONSOLE=1` und `BuildConfig::SenderWebConsoleEnabled` aktiv sind.

## Zentrale Konfiguration

Die statische Konfiguration ist in einzelne Header unter `include/config/` aufgeteilt. Runtime-Zust�nde liegen nicht in Config-Dateien. Die wichtigsten Bereiche:

- `NetworkConfig`: SSIDs, Passw�rter, Webserver-Port, OTA-Hostnamen, ESP-NOW-Kanal, ESP-NOW-AES-Key und feste Peer-MAC-Adressen.
- `DisplayConfig`: Display-Power-Pin, Backlight-Pin, Button-Pin, Rotation, Seitenanzahl, UI-Timeouts, Farben und Grenzwerte.
- `SenderConfig`: CAN-/OBD2-Pins, Polling-/Timeout-Werte, UDS-Timing, Heartbeat und Power-Messung.
- `BuildConfig`, `SimulationConfig` und `SecurityConfig`: Build-Flags, Simulation, OTA/Web-Feature-Flags und Schutz der Web-Endpunkte.

Aktuelle WLAN-/OTA-Werte werden nicht mehr fest im Repository gepflegt. Für echte Geräte `include/secrets.example.h` nach `include/secrets.h` kopieren und dort setzen:

| Ziel | SSID | Passwort | Zweck |
| --- | --- | --- | --- |
| Sender | `Secrets::SenderWebSsid` | `Secrets::SenderWebPassword` | WebConsole + Web-OTA |
| Display | `Secrets::DisplayWebSsid` | `Secrets::DisplayWebPassword` | Display-Web-OTA |

ESP-NOW nutzt zentral `NetworkConfig::EspNowChannel`. Wenn Web-OTA/AP und ESP-NOW parallel laufen, müssen beide Geräte auf demselben Kanal bleiben.

### Vorlaeufiger Debug-Modus mit `secrets.example.h`

Fuer die aktuelle Inbetriebnahme ist in `include/config/SecurityConfig.h`
vorlaeufig gesetzt:

```cpp
SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets = false
```

Damit starten Sender- und Display-WLAN sowie ESP-NOW auch dann, wenn noch keine
lokale `include/secrets.h` existiert und der Build auf `include/secrets.example.h`
zurueckfaellt. Das ist nuetzlich, um nach dem Flashen die Weboberflaechen und
ESP-NOW-Verbindung ueberhaupt testen zu koennen.

Wichtig: Dieser Modus ist nur fuer Debug/Inbetriebnahme gedacht. Die Beispiel-
Passwoerter aus `secrets.example.h` sind bekannt und duerfen im normalen Betrieb
nicht verwendet werden. Fuer reale Nutzung:

1. `include/secrets.example.h` nach `include/secrets.h` kopieren.
2. AP-Passwoerter, Web-Passwort, API-Token und ESP-NOW-Key aendern.
3. Sender-/Display-MAC-Adressen pruefen.
4. `BlockNetworkFeaturesOnPlaceholderSecrets` wieder auf `true` setzen.

### Hinweis: Display bleibt nach Upload schwarz

Beim LilyGO T-Display S3 wird die Displayversorgung auf vielen Revisionen über GPIO 15 eingeschaltet. Dieser Wert ist jetzt zentral gesetzt:

```cpp
DisplayConfig::PowerPin = 15
DisplayConfig::BacklightPin = 38
```

Falls eine andere Board-Revision verwendet wird und das Display trotzdem dunkel bleibt, zuerst prüfen:

1. Wurde wirklich `env:display` auf das LilyGO T-Display S3 geflasht?
Die statische Konfiguration ist in einzelne Header unter `include/config/` aufgeteilt. Runtime-Zust�nde liegen nicht in Config-Dateien. Die wichtigsten Bereiche:
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
- Diag: PID-Support, letzter CAN-Zeitstempel, VIN, DTCs, OBD-ZÃ¤hler,
  letzte OBD-Anfrage, letzte ECU-Antwort, letzte Negative Response und
  herunterladbarer Diagnose-Snapshot
- Log: Live-Log und persistenter Sender-Diagnose-Log
- OTA: Firmwaredatei `firmware.bin` direkt über die Webseite hochladen

Standardmäßig wartet der Sender nicht mehr auf die WebConsole. Nur wenn
`SenderConfig::RequireWebStart = true` gesetzt wird, dient der Button
`Sender starten` als manuelle Freigabe. OTA, WebConsole und Heartbeat bleiben
auch in diesem manuellen Modus aktiv.

### Sender-Capability-Scanner

Der neue Tab `Cap` in der Sender-WebConsole ist fuer gezielte Fahrzeugtests
gedacht und startet nie automatisch. Waehrend ein Capability-Scan aktiv ist,
pausiert der Sender die normalen OBD-/UDS-Liveabfragen, damit sich die
ISO-TP-Antworten nicht gegenseitig stoeren.

Verfuegbare Aktionen:

| Methode | Pfad | Zweck |
| --- | --- | --- |
| `GET` | `/api/capabilities/status` | aktueller Scanstatus mit Tabellen-Daten |
| `POST` | `/api/capabilities/obd/start` | OBD Mode-01 PID-Scan starten |
| `POST` | `/api/capabilities/uds/start` | UDS ECU-/DID-Scan starten |
| `POST` | `/api/capabilities/can/start` | passiven CAN-Sniffer vormerken |
| `POST` | `/api/capabilities/can/baseline` | CAN-Sniffer-Baseline und Kandidaten neu setzen |
| `POST` | `/api/capabilities/stop` | laufenden Scan stoppen |
| `GET` | `/api/capabilities/export.json` | Scan-Ergebnis als JSON herunterladen |

Der OBD-Scan fragt zuerst die Supported-PID-Masken `0x00`, `0x20`, `0x40`
und bei Bedarf `0x60` ab. Danach werden die wichtigen Live-PIDs testweise
angefragt, darunter Drehzahl, Geschwindigkeit, Kuehlmitteltemperatur, MAP,
Baro, MAF, Tankfuellstand, ECU-Spannung, Umgebungstemperatur, Oeltemperatur
und Kraftstoffrate. Die Webtabelle unterscheidet:

- `OK`: PID wird unterstuetzt, antwortet und laesst sich dekodieren.
- `UNSUPPORTED`: PID ist laut Supported-Mask nicht verfuegbar.
- `TIMEOUT`: PID ist gemeldet oder angefragt, aber es kam keine Antwort.
- `DECODE_ERROR`: Antwort kam, passt aber nicht zum bekannten Datenformat.
- `NEGATIVE_RESPONSE`: ECU hat die Anfrage aktiv abgelehnt.

Der UDS-Scan nutzt eine zentrale, konservative Ziel-Config
`include/config/UdsConfig.h`. Standardmaessig werden nur physische, lesende
Diagnoseziele gescannt, z. B. Motor `0x7E0 -> 0x7E8`, Getriebe
`0x7E1 -> 0x7E9`, ABS/ESP `0x7E2 -> 0x7EA`, BCM `0x7E4 -> 0x7EC`,
Klima `0x746 -> 0x7B0` und Kombiinstrument `0x714 -> 0x77E`.
`0x7DF` bleibt als funktionaler OBD-/Gateway-Kontext dokumentiert, wird aber
nicht als physisches UDS-Scan-Ziel benutzt.

UDS ist absichtlich read-only ausgelegt. `SecurityAccess`, Schreibdienste,
Codierung, Anpassung und Loeschen von Fehlern sind deaktiviert. Der Scan nutzt
lesende Dienste wie `0x3E TesterPresent` und `0x22 ReadDataByIdentifier`.
Standardmaessig gepruefte DIDs sind vor allem Identifikationswerte wie
`0xF180` bis `0xF190` inklusive VIN sowie `0xF195`. VW-spezifische Live-DID-
Kandidaten, z. B. fuer Oeltemperatur oder Ladedruck, sind in der Config
dokumentiert, aber nicht automatisch im Default-Scan aktiv, weil Bedeutung und
Skalierung vom konkreten Steuergeraet abhaengen. NRC `0x78`
(`ResponsePending`) wird als Zwischenstatus behandelt: der Client wartet bis
zum konfigurierten Gesamtfenster `SenderConfig::UdsResponsePendingTimeoutMs`
weiter. Erst danach wird der Test als Timeout bewertet.

Der CAN-Signal-Finder ist passiv an den `CanRouterHub` angebunden. Er sendet
keine Frames und beeinflusst keine Steuergeraete. Gelesene Roh-CAN-Frames
werden von `CANHandler::processIncoming()` an registrierte Listener verteilt.
Der Finder arbeitet als Baseline-/Aktionsvergleich und sammelt daraus
geaenderte Bytes, Bitmasken, Wechselzaehler und eine einfache Confidence-
Bewertung als Kandidaten. Damit lassen sich beliebige beobachtbare Aktionen
wie Blinker, Gangwechsel, Licht, Tueren oder Assistenzzustaende eingrenzen,
ohne einen zweiten TWAI-Leser zu starten.

Fuer die Suche nach Ereignissen empfiehlt sich:

1. CAN Signal-Finder starten.
2. Ruhigen Ausgangszustand herstellen.
3. `Baseline aufnehmen` druecken und ein paar Sekunden warten.
4. `Aktion starten` druecken.
5. Genau ein Ereignis ausloesen, z. B. Blinker links.
6. `Aktion stoppen / analysieren` druecken.
7. Kandidatenliste und Export-JSON sichern.

### Sender-Diagnose-Log am OBD2-Anschluss

Der Sender schreibt zusätzlich zum seriellen Monitor einen persistenten
Diagnose-Log in SPIFFS. Damit lassen sich Probleme nach einer Fahrt auswerten,
auch wenn kein Laptop am seriellen Monitor hing.

Der Log enthält unter anderem:

- Boot-Informationen mit Firmware-, Target- und Protokollversion
- ESP-NOW Heartbeats, Sendefehler und gesendete Telemetrie-Payloads
- CAN-/TWAI-Status und CAN-Rohframes
- OBD-Rohantworten, dekodierte OBD-Werte und OBD-Timeouts
- Supported-PID-Erkennung
- automatische Deaktivierung nicht unterstÃ¼tzter PIDs nach erfolgreicher
  Supported-PID-Abfrage
- Fallback von funktionaler Anfrage `0x7DF` auf physische Anfrage `0x7E0`,
  wenn mehrere OBD-Timeouts hintereinander auftreten
- Negative Responses inklusive NRC-Code und Kurzbeschreibung
- VIN-Abfrage Ã¼ber Mode 09 PID 02
- DTC-Abfragen und erkannte Fehlercodes
- OTA-, Simulation- und Neustart-Ereignisse aus der WebConsole

Abruf über die Sender-WebConsole:

| Methode | Pfad | Zweck |
| --- | --- | --- |
| `GET` | `/log` | aktueller RAM-Live-Log |
| `GET` | `/log/file` | vollständiger persistenter Diagnose-Log inklusive Archiv |
| `POST` | `/api/log/clear` | RAM- und persistenten Log löschen |
| `GET` | `/status` | enthält `diagnosticLogMounted`, `diagnosticLogSize`, `diagnosticLogMaxSize` |

Zusätzliche OBD-Diagnose-Endpunkte:

| Methode | Pfad | Zweck |
| --- | --- | --- |
| `GET` | `/api/diagnostic/snapshot` | strukturierter JSON-Snapshot mit CAN/OBD/ESP-NOW-Zählern |
| `GET` | `/api/diagnostic/download` | JSON-Snapshot als Datei herunterladen |

Wichtige Diagnosefelder im Snapshot:

- `obd.requests`: Anzahl gesendeter OBD-/ISO-TP-Anfragen
- `obd.validResponses`: gültige ECU-Antworten
- `obd.timeouts`: Anfragen ohne ECU-Antwort
- `obd.negativeResponses`: ECU-Ablehnungen mit NRC
- `obd.timeoutStreak`: aktuelle Timeout-Serie
- `obd.requestCanId`: aktuell verwendete Anfrage-ID, normalerweise `0x7DF`,
  nach Fallback `0x7E0`
- `obd.lastRequest`: letzte OBD-Anfrage
- `obd.lastEcuResponse`: letzte ECU-Antwort oder Timeout/NRC
- `supportedPids`: erkannte Supported-PID-Masks
- `vehicle.vin` und `vehicle.dtc`: zuletzt gelesene VIN und Fehlercodes

### UDS-Diagnose

Der Sender besitzt zusätzlich ein erstes UDS-Modul nach ISO 14229 auf derselben
ISO-TP-Schicht wie OBD2. Es ist bewusst auf lesende bzw. ungefährliche
Diagnosefunktionen begrenzt:

- `0x22 ReadDataByIdentifier`, aktuell DID `0xF190` für VIN/Identifikation
- `0x19 ReadDTCInformation`, Subfunktion `0x02` mit Statusmaske `0xFF`
- `0x3E TesterPresent`
- `0x10 DiagnosticSessionControl` ist im UDS-Client vorbereitet, aber nur als
  Default-Session-Aufruf erlaubt

Aus Sicherheitsgründen blockiert der UDS-Client aktuell:

- `0x11 ECUReset`
- `0x27 SecurityAccess`
- Schreib-, Routine-, Codier- und Flash-Funktionen

UDS nutzt physische Anfrage-ID `0x7E0` und erwartet Antworten im Bereich
`0x7E8` bis `0x7EF`. Die Sender-WebConsole zeigt im Diagnosebereich:

- UDS verfügbar ja/nein
- UDS Requests, Timeouts, Sendefehler, positive und negative Antworten
- letzte UDS-Anfrage
- letzte UDS-Antwort
- letzte UDS Negative Response inklusive NRC
- letzter gelesener DID-Wert
- letzte UDS-DTC-Zusammenfassung

Dieselben Informationen stehen im JSON unter `/status` und im Snapshot unter
`/api/diagnostic/snapshot` im Objekt `uds`.

Die Loggröße ist über `SenderConfig::DiagnosticLogMaxBytes` begrenzt. Bei
Überschreitung wird die aktuelle Datei nach
`SenderConfig::DiagnosticLogArchivePath` rotiert und eine neue Logdatei
begonnen. Telemetrie-Payloads werden über
`SenderConfig::PersistTelemetryPayloadsToDiagnosticLog` mitgeschrieben. Für
lange Dauerfahrten kann dieses Flag auf `false` gesetzt werden, um Flash-Writes
zu reduzieren.

### Display-Diagnose-Log

Das Display nutzt dieselbe persistente Diagnose-Log-Schicht wie der Sender,
schreibt aber in eigene Dateien:

- `/display_diagnostic.log`
- `/display_diagnostic.old`

Die Display-Weboberfläche zeigt Logstatus und Größe an und bietet:

| Methode | Pfad | Zweck |
| --- | --- | --- |
| `GET` | `/log/file` | vollständiger persistenter Display-Log |
| `GET` | `/log/download` | Display-Log als Datei herunterladen |
| `POST` | `/api/log/clear` | Display-Log löschen |

Im Display-Log landen unter anderem Bootdaten, Web-OTA-Ereignisse,
Simulation-Schalter, ESP-NOW-Heartbeat-Empfang, RX-Statistiken, UI-Seitenwechsel
und ESP-NOW-Timeouts. Die Pfade und Größen liegen zentral in
`include/LoggingConfig.h`.

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

Bei Pushes auf `main`/`master`, Pull Requests und manuellen Runs baut GitHub Actions automatisch:

- `env:sender`
- `env:display`

`VERSION.txt` bleibt dabei unverändert und liefert die Basisversion. CI ergänzt
für normale Build-Läufe einen Suffix (`-ci.<run>` bzw. `-pr<nr>.<run>`), damit
Builds reproduzierbar bleiben und der Workflow keine Commits in den Branch
zurückschreibt.

Die erzeugten Dateien liegen danach als GitHub-Actions-Artefakt vor. Ein
GitHub Release wird nicht im normalen CI-Build erzeugt; siehe Abschnitt
`GitHub Release erstellen`:

- `CAN_OBD2_sender_<VERSION>.bin`
- `CAN_OBD2_display_<VERSION>.bin`
- `CAN_OBD2_sender_<VERSION>.elf`
- `CAN_OBD2_display_<VERSION>.elf`
- `firmware_manifest.json`

Für Web-OTA vom iPhone werden normalerweise nur die `.bin`-Dateien benötigt.

Hinweis: GitHub Actions kopiert fuer CI- und Release-Builds automatisch
`include/secrets.example.h` nach `include/secrets.h`. Damit bauen Sender und
Display reproduzierbar mit den Beispielpasswoertern aus dem Repository. Reale
Geraete sollten weiterhin mit einer lokalen, nicht eingecheckten
`include/secrets.h` gebaut werden.

## GitHub Release erstellen

Releases sind vom normalen CI-Build getrennt. Dadurch erzeugt ein normaler Push
keine Tags und keine Releases mehr.

1. In GitHub `Actions` oeffnen.
2. Workflow `Release Firmware` auswaehlen.
3. `Run workflow` anklicken.
4. Version ohne fuehrendes `V` eingeben, z. B. `1.0.14`.
5. Optional `prerelease` oder `draft` aktivieren.

Der Workflow baut Sender und Display, fuehrt die Native-Tests aus, erstellt den
Tag `firmware-V<version>` und veroeffentlicht danach ein GitHub Release mit den
Firmware-Artefakten.

## Telemetrie-Protokoll

Sender und Display verwenden ein gemeinsames CRC-geschütztes ESP-NOW-Binärpaket:

- `lib/telemetry/TelemetryPacket.h`
- `lib/telemetry/TelemetryCodec.h`
- `lib/telemetry/TelemetrySequence.h`
- `lib/transport/EspNowTelemetryTransport.h`

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
Der Sender verschickt kodierte Pakete über den gemeinsamen
`Transport::sendTelemetryPacket()`-Pfad, damit es keine versteckten separaten
`esp_now_send()`-Implementierungen für Telemetrie gibt.

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

Aktuelle Firmware-Reihenfolge:

1. Hauptanzeige
2. Motorwerte
3. Verbrauch / Trip
4. Diagnose kompakt
5. UDS / Fehlercodes
6. CAN-Sniffer
7. RPM-Bogen
8. Zusatzwerte

1. Hauptanzeige: Geschwindigkeit, Drehzahl, Kühlmitteltemperatur, Batteriespannung
2. Motorwerte: Öltemperatur, Kühlmitteltemperatur, Motorlast, Ansauglufttemperatur
3. Verbrauch: Durchschnittsverbrauch, Kraftstoffrate, Geschwindigkeit, Drosselklappe
4. Zusatzwerte: MAF, Tankfüllstand, Motorlaufzeit, Umgebungstemperatur
5. CAN-Rohdaten: letzter CAN-Frame, Frame-Zähler, einfache OBD/CAN-Hinweise
6. Diagnose: ESP-NOW, CAN, OBD, Heartbeat, Datenqualität und Paketverlust
7. Fehlercodes: DTC-Status und aktive Fehlercodes
8. Drehzahl-Grafik: grosser RPM-Wert, Balken 0 bis `DisplayConfig::RpmMax`, Warn-/Kritisch-Marken und Max-RPM seit Start

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
- `PowerRunning`
- `PowerStartStop`
- `PowerIdle`
- `PowerParked`
- `PowerDisplaySleep`
- `PowerWakeup`

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

Sender-LEDs:

- `LedPin1` ist die grüne Betriebs-LED. Sie bedeutet primär: Sender-Firmware lebt.
- `LedPin2` ist die Fehler-/Status-LED.
- Die LED-Statusmaschine liegt zentral in `lib/status/SenderLedController.*`; direkte LED-Pin-Schreibzugriffe sollen nur im Sender-LED-Hardwarewrapper erfolgen.
- Nach echtem CAN-Verkehr und anschließendem CAN-Idle wechselt der Status auf `VehicleOff`. Sobald wieder CAN-Verkehr oder OBD-Antworten eintreffen, schaltet der Controller zurück auf `CanActive`/`ObdActive` und die grüne LED geht wieder an.
- Die Sender-WebConsole zeigt den aktuellen LED-Zustand, LED-Test aktiv/inaktiv, `VehicleOff` und den letzten LED-Zustandswechsel im Statusbereich und in `/status`.

## Power Management / Start-Stopp

Der Sender entscheidet zentral ueber den Fahrzeug-Energiezustand. Moderne Start-Stopp-Fahrzeuge werden nicht nur ueber Drehzahl oder Bordspannung bewertet, sondern ueber einen Activity Score aus CAN, OBD, RPM, Geschwindigkeit, Batteriespannung, Motorlast, Drosselklappe, Benutzeraktivitaet und Simulation.

Zustaende:

- `Booting`: Firmware startet.
- `Running`: Motor laeuft oder Fahrzeug bewegt sich.
- `StartStop`: RPM = 0 und Geschwindigkeit = 0, aber CAN, OBD und Bordnetz sind aktiv.
- `Idle`: Fahrzeug steht, Bus/OBD/Bordnetz sind noch aktiv; Display bleibt wach.
- `Parked`: CAN und OBD sind laenger als `PowerConfig::ParkDetectionTimeoutMs` inaktiv.
- `DisplaySleep`: erst nach `Parked` plus `PowerConfig::DisplaySleepAfterMs`.

Wichtig fuer Fahrzeuge mit Start-Stopp: `StartStop` startet keinen Display-Sleep-Timer. OBD-Abfragen und ESP-NOW-Heartbeat laufen weiter.

Konfiguration:

- `include/config/PowerConfig.h`
- aktives Profil aktuell: `PowerConfig::VehiclePowerProfile::VolkswagenMQBEvo`
- `ParkDetectionTimeoutMs = 5 Minuten`
- `DisplaySleepAfterMs = 10 Minuten`

Datenfluss:

```text
CAN / OBD / Werte / Benutzeraktivitaet
              |
              v
        ActivityMonitor
              |
              v
         VehicleState
              |
              v
       PowerCommand via ESP-NOW
              |
              v
        Display Power Page
```

Sender-WebConsole:

- Statuskarte `Power Manager`
- `/status` Felder: `vehicleState`, `activityScore`, `powerCommand`, `startStopDetected`, `parkedDetected`, `displaySleepDueAtMs`

Display:

- Seite `Power Management`
- zeigt VehicleState, Activity Score, PowerCommand, Display-Zustand sowie letzte CAN-/OBD-Aktivitaet.
- Bei `POWER_COMMAND=Sleep` wird das Backlight ausgeschaltet und die UI pausiert.
- Bei `POWER_COMMAND=Wakeup` oder Display-Tastendruck wird das Display wieder aktiviert.

Power-Simulationsszenarien:

- `PowerRunning`
- `PowerStartStop`
- `PowerIdle`
- `PowerParked`
- `PowerDisplaySleep`
- `PowerWakeup`
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
Wenn Platzhalterwerte aus `include/secrets.example.h` noch aktiv sind, blockiert
die Firmware sicherheitsrelevante Netzwerkpfade gezielt:

- Sender-WebConsole und Sender-OTA starten nicht mit Platzhalter-Passwörtern
  oder Platzhalter-API-Token.
- Display-Web-OTA startet nicht mit Platzhalter-Passwörtern oder
  Platzhalter-API-Token.
- ESP-NOW Sender/Empfänger starten nicht mit dem öffentlichen Beispiel-AES-Key.

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
- Display `macdrop` steigt: Sender-MAC passt nicht zu `NetworkConfig::SenderAllowedMac`.
- Display `crc` steigt: AES-Key/Kanal oder Frame-Integritaet pruefen.
