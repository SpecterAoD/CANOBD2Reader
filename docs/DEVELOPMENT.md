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

`VERSION.txt` enthält die SemVer-Version für lokale Builds (`V2.1.0-beta.1`).
Release-Builds setzen sie über `CANOBD2_FIRMWARE_VERSION`
(`scripts/apply_firmware_version.py`).

## Firmware auf ein Gerät per WLAN

```bash
python scripts/ota_upload.py --host 192.168.4.1 output/sender.bin
# lokaler Build, vorher signieren:
python scripts/ota_upload.py --sign-key ~/canobd2-signing.pem .pio/build/sender/firmware.bin
```

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
