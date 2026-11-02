# Architektur

## Überblick

```
Fahrzeug (OBD-II / CAN 500 kbit/s)
        │  TWAI (ESP32)
        ▼
┌──────────────────────┐   ESP-NOW (AES, Kanal 1)   ┌──────────────────────┐
│ Sender (ESP32)       │ ─────────────────────────▶ │ Display (T-Display S3)│
│  OBD-Poller, UDS,    │   Live-Snapshot 10×/s      │  Sprite-Renderer,     │
│  Power-Manager, LEDs │   + Status-/Diagnosetexte  │  8 Seiten, Taster     │
│  Management-AP + Web │                            │  Management-AP + Web  │
└──────────────────────┘                            └──────────────────────┘
        ▲                                                     ▲
        └──── signierte Updates: Web-Upload oder GitHub ──────┘
```

Beide Geräte sind eigenständige PlatformIO-Environments (`sender`, `display`).
Gemeinsamer Code liegt in `lib/` und wird nativ getestet (`env:native`).

## Datenfluss Livewerte

1. `Obd::PidPollPlanner` wählt pro Durchlauf den PID, der relativ zu seinem
   Sollintervall am stärksten überfällig ist (schnell 200 ms: Drehzahl, Tempo,
   Last, Luftmasse; langsam 2 s: Temperaturen, Spannung, Tank …). Nicht
   unterstützte PIDs (laut Maske des Motorsteuergeräts 0x7E8) werden übersprungen.
   Antworten anderer Steuergeräte (z. B. Getriebe 0x7E9) werden für Mode 01/09 verworfen.
   Der Verbrauch wird aus der Luftmasse berechnet (PID 0x5E ist bei VW unzuverlässig).
2. `OBD2Handler` dekodiert die Antwort und schreibt sie in die
   `LiveData::LiveValueTable` (`SenderLiveData`). Abgeleitete Werte
   (Ladedruck, Verbrauch) und die Bordspannung landen in derselben Tabelle.
3. Alle 100 ms wird die komplette Tabelle als ein binäres Paket gesendet
   (Protokoll v3, siehe [PROTOCOL.md](PROTOCOL.md)).
4. Das Display übernimmt den Snapshot in `DisplayLogic::LiveValueView`. Jeder
   Wert ist *frisch*, *veraltet* (grau, letzter Wert bleibt sichtbar),
   *nicht unterstützt* oder *fehlend*.
5. `DisplayRenderer` zeichnet alle 50 ms ein komplettes Bild in einen Sprite und
   überträgt es in einem Stück. Dadurch flackert nichts.

Statusinformationen (CAN/OBD-Zustand, DTC, VIN, UDS, Power-Zustand,
Heartbeat) sind selten und bleiben Textpakete.

## Verzeichnisse

| Pfad | Inhalt |
|---|---|
| `src/sender/` | Sender-Firmware: App, Scheduler (OBD, UDS, Power, Simulation), CAN-Treiber, WebConsole |
| `src/display/` | Display-Firmware: Empfang, Datenmodell, Renderer, Weboberfläche |
| `lib/telemetry/` | Paketformat, CRC-Codec, Live-Snapshot (`LiveData`) |
| `lib/isotp/`, `lib/obd/`, `lib/uds/` | ISO-TP, OBD-Dekoder, PID-Planer, UDS-Client |
| `lib/display/` | Live-Wert-Klassifizierung, Glättung, Schweregrade |
| `lib/power/`, `lib/status/` | Aktivitätserkennung (Start-Stopp, Parken, Display-Schlaf), LED-Zustände |
| `lib/update/` | GitHub-Manifest, SemVer, `OtaSession` (signierte Installation), Boot-Validierung |
| `lib/firmware_signing/`, `lib/monocypher/` | Ed25519-Prüfung der Firmware |
| `lib/security/` | Gerätezugangsdaten im NVS |
| `lib/diagnostics/` | Neustartgrund, Absturzbericht (Coredump), Loop-Watchdog |
| `lib/web/` | Gemeinsamer Management-Webserver, Authentifizierung, HTML-Assets (`DevicePages`, `WebAssets`) |
| `lib/simulation/`, `lib/capabilities/`, `lib/can_router/` | Entwicklerwerkzeuge (nur `*_dev`-Builds aktiv) |
| `include/config/` | Konfiguration je Bereich (keine Zugangsdaten) |
| `scripts/` | Signieren, Upload, Release-Manifest, Secret-Check, Build-Hooks |
| `test/` | Native Unit-Tests (Unity) und `test/python` (pytest) |

## Entwicklerwerkzeuge

`CANOBD2_DEV_TOOLS=1` (Environments `sender_dev`, `display_dev`) aktiviert
Simulation, OBD/UDS-Capability-Scanner, CAN-Signal-Finder, RAW-CAN-Telemetrie
und die CAN-Sniffer-Seite. In Release-Builds sind Endpunkte und Bedienelemente
nicht vorhanden; die Simulation kann dort nicht eingeschaltet werden.

Display-Hardware, Framebuffer und Taster liegen in `DisplayUi`; Seiten und Widgets
in `DisplayRenderer`. Web-Routen verwenden die separat abgelegten Seiten aus
`lib/web/DevicePages.cpp`. JSON-Escaping teilen sie mit dem Update- und WLAN-Code
über `lib/utils/JsonEscape.h`; Browser-HTML-Escaping kommt aus `WebAssets::commonScript`.
`ConsumptionAccumulator` integriert Kraftstoffmenge und Strecke unabhängig von
PID-Abfragefrequenz und Display-Rendering. `UpdateSafety` trennt unbekannten,
geparkten und aktiven Fahrzeugzustand vom ausdrücklich bestätigten Wartungsmodus.
