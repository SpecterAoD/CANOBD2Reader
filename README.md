# CANOBD2Reader

Ein ESP32 liest OBD-II-Livewerte über CAN aus und schickt sie verschlüsselt
per ESP-NOW an ein LilyGO T-Display S3 im Auto.

- **Sender** (`env:sender`): ESP32 DevKit V1 mit CAN-Shield. OBD-II-Poller,
  UDS-Diagnose (nur lesend), Power-Manager (Start-Stopp, Parken, Display-Schlaf), Weboberfläche.
- **Display** (`env:display`): flackerfreie Anzeige mit 4 Fahrseiten und
  4 Diagnoseseiten, Weboberfläche.

Stand: `V2.1.0-beta.1`, Telemetrie-Protokoll 3.

## Schnellstart

```bash
pip install -r requirements-dev.txt
python scripts/make_secrets.py --sender-mac <MAC> --display-mac <MAC>
pio run -e sender -t upload
pio run -e display -t upload
```

Ausführlich, auch für das Flashen von einem anderen Linux-Rechner aus:
[docs/PROVISIONING.md](docs/PROVISIONING.md).

Danach mit dem WLAN `CANOBD2_Sender` bzw. `CANOBD2_Display` verbinden und
`http://192.168.4.1` öffnen.

## Wichtige Eigenschaften

- **Livewerte:** Alle Werte gehen als ein binärer Snapshot 10× pro Sekunde
  zum Display. Die PID-Abfrage ist fair: schnelle Werte etwa alle 200 ms,
  langsame alle 2 s. Ein ausgebliebener Wert wird grau statt leer.
- **Signierte Updates:** Web-Upload und GitHub-Update installieren nur
  Ed25519-signierte Firmware. Neue Firmware, die abstürzt, wird automatisch
  zurückgerollt.
- **Keine Zugangsdaten im Repository:** Jedes Gerät speichert sie im NVS.
  Release-Images enthalten keine.
- **Entwicklerwerkzeuge** (Simulation, Capability-Scanner, CAN-Signal-Finder)
  nur in `sender_dev` / `display_dev`.

## Dokumentation

| Datei | Inhalt |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Aufbau, Datenfluss, Verzeichnisse |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | Funkprotokoll v3 |
| [docs/PROVISIONING.md](docs/PROVISIONING.md) | Erstinstallation / Umstieg per USB |
| [docs/SECURITY.md](docs/SECURITY.md) | Zugangsdaten, Signaturen, Schlüssel, Rollback |
| [docs/HARDWARE.md](docs/HARDWARE.md) | Pins, LEDs, Display-Seiten, Inbetriebnahme |
| [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) | Build, Tests, Release |

## Lizenz

Siehe [LICENSE](LICENSE). Enthält Monocypher 4.0.2 (BSD-2-Clause/CC0,
`lib/monocypher`). Ursprünglich basierend auf dem
[MrDIY CAN-Bus Gateway](https://gitlab.com/MrDIYca/canabus).
