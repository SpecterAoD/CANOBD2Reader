# Entwicklung

## Einrichtung

```bash
python -m venv .venv && . .venv/bin/activate   # Windows: .venv\Scripts\activate
pip install -r requirements-dev.txt
```

## Environments

| Environment | Zweck |
|---|---|
| `sender`, `display` | Release-Firmware (ohne Entwicklerwerkzeuge) |
| `sender_dev`, `display_dev` | mit Simulation, Capability-Scanner, CAN-Sniffer |
| `native` | Unit-Tests auf dem PC |

```bash
pio run -e sender -t upload          # per USB flashen (Port wird automatisch gesucht)
pio device monitor -e sender
pio test -e native                   # C++-Unit-Tests
python -m pytest test/python         # Skript-Tests
python scripts/check_secrets.py      # vor jedem Commit
```

Die Plattform ist fest auf `espressif32 @ 8.0.0` (Arduino-ESP32 3.3.12) gesetzt.
Ein Plattform-Update ist eine bewusste Änderung und muss auf beiden Geräten
getestet werden.

## Version

`VERSION.txt` enthält die SemVer-Version für lokale Builds (`V2.1.1-beta.4`).
Release-Builds setzen sie über `CANOBD2_FIRMWARE_VERSION`
(`scripts/apply_firmware_version.py`).

## Firmware auf ein Gerät per WLAN

```bash
python scripts/ota_upload.py --host 192.168.4.1 output/sender.bin
# lokaler Build, vorher signieren:
python scripts/ota_upload.py --sign-key ~/canobd2-signing.pem .pio/build/sender/firmware.bin
```

## Logs per USB auslesen

Ohne WLAN, direkt über USB (nur lesend). Das Board kann am selben Rechner oder
an einem per SSH erreichbaren Linux-Rechner hängen:

```bash
python scripts/read_device_logs.py sender --remote kalipi --out ~/canobd2-logs
python scripts/read_device_logs.py display --remote kalipi --out ~/canobd2-logs
```

Geräte mit dem alten Layout (bis V2.1.0-beta.2): zusätzlich `--layout legacy`.
**Vor** einem Layout-Wechsel auslesen, sonst ist das alte Log weg. Die Logs
können die Fahrgestellnummer enthalten, also nicht ins Repository übernehmen.

## Abstürze analysieren

Jeder Start schreibt den Grund ins Diagnose-Log (`[BOOT] … reset=PANIC|TASK_WDT|…`).
Nach einem Absturz liegt ein Absturzbericht (Coredump, ELF) im Flash. Man lädt
ihn in der Weboberfläche oder per `GET /api/coredump` herunter und wertet ihn
mit der `firmware.elf` **genau dieses Builds** aus (Release-Anhang `debug-sender.zip` bzw. `debug-display.zip` oder
CI-Artefakt **desselben Builds**; ein Neubau desselben Commits ist kein Ersatz):

```bash
pip install esp-coredump
esp-coredump info_corefile -t raw -c CANOBD2_sender_…_coredump.bin .pio/build/sender/firmware.elf
```

Danach den Bericht in der Weboberfläche löschen (`POST /api/coredump/clear`).

## Release

GitHub → Actions → **Release** → Version (z. B. `2.1.0-beta.1`) und Kanal
wählen. Der Workflow testet, baut, signiert, prüft die Signatur, erweitert das
Update-Manifest und veröffentlicht beides. Voraussetzung ist das Secret
`FIRMWARE_SIGNING_KEY` im Environment `release` (siehe [SECURITY.md](SECURITY.md)).

## Regeln

* Protokolländerung → `CANOBD2_PROTOCOL_VERSION` erhöhen, beide Geräte gemeinsam aktualisieren.
* Neue Livewerte nur hinten in `LiveData::Metric` anfügen.
* Gemeinsame Logik nach `lib/` mit nativem Test, Geräte-Code nach `src/`.
* Keine Zugangsdaten in Code, Tests oder Doku.

## Release-Artefakte und Indexschutz (ab V2.1.1-beta.4)

CI archiviert BIN, ELF, MAP, Konfiguration, Paketversionen und SHA-256-Prüfsummen
für alle vier Environments für 90 Tage. Releases behalten `debug-sender.zip`
und `debug-display.zip` als Anhänge. `build.json` enthält Commit, Version,
Environment und die konkret installierten PlatformIO-Paketversionen.
Für Absturzanalysen immer das Archiv der tatsächlich installierten Firmware nutzen.

Der Release-Workflow lädt und validiert den vorhandenen Index mit drei begrenzten
Versuchen. Netzwerkfehler, beschädigtes JSON und unerwartete HTTP-Antworten brechen
ab. Nur bei HTTP 404 und explizitem `initialize_index=true` wird erstmals ein Index
angelegt. `previous_manifest.json` bleibt als Release-Anhang erhalten; nach einer
Fehlveröffentlichung kann dieser geprüfte Stand nach Freigabe wieder als Index
hochgeladen werden. Alte Firmware-Dateien werden dabei nicht gelöscht.
