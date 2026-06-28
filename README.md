# CANOBD2Reader

ESP32-basierter CAN-/OBD2-Reader mit ESP-NOW-Telemetrie und separater LilyGo-T-Display-S3-Anzeige.

## Sketches

- `ino/CAN_OBD2_Gateway.ino`: Reader/Sender für CAN, OBD2-Abfragen, Batteriespannung und ESP-NOW-Telemetrie.
- `ino/Anzeige_LilyGoTDisplayS3.ino`: Anzeige/Empfänger mit mehreren Dashboard-Seiten.

## Telemetrie-Protokoll

Textpakete werden als CRC-geschützter ESP-NOW-Frame übertragen:

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

Die Anzeige markiert fehlende oder veraltete Werte nach Timeout als `--`.

Zusätzlich fragt der Sender unterstützte PID-Bereiche ab und überspringt nicht unterstützte Live-Daten-PIDs. Fehlercodes werden über OBD Mode `0x03` gelesen und als `DTC`-Paket übertragen.

## Display-Seiten

1. Hauptanzeige: Geschwindigkeit, Drehzahl, Kühlmitteltemperatur, Batteriespannung
2. Motorwerte: Öltemperatur, Kühlmitteltemperatur, Motorlast, Ansauglufttemperatur
3. Verbrauch: Durchschnittsverbrauch, Kraftstoffrate, Geschwindigkeit, Drosselklappe
4. Zusatzwerte: MAF, Tankfüllstand, Motorlaufzeit, Umgebungstemperatur
5. CAN-Rohdaten: letzter CAN-Frame, Frame-Zähler, einfache OBD/CAN-Hinweise
6. Diagnose: ESP-NOW, CAN/OBD, Datenqualität, CRC-/Drop-Zähler
7. Fehlercodes: DTC-Status und aktive Fehlercodes

## Simulation

Für Kommunikationstests ohne Fahrzeug kann im Sender `ENABLE_TELEMETRY_SIMULATION` auf `1` gesetzt werden. Dann initialisiert der Sender nur ESP-NOW und sendet simulierte Werte für alle Display-Seiten.

Für reine Displaytests ohne Sender kann im Display `ENABLE_DISPLAY_INTERNAL_SIMULATION` auf `1` gesetzt werden. Der normale Testpfad sollte aber der Sender-Simulationsmodus sein, weil damit ESP-NOW, CRC, Paketformat und Anzeige gemeinsam geprüft werden.

Gemeinsame Logik liegt in:

- `include/TelemetryProtocol.h`
- `include/SimulationData.h`
- `include/CANDecoder.h`
- `src/CANDecoder.cpp`

## OTA-Hinweis

`platformio.ini` nutzt `board_build.partitions = min_spiffs.csv`. Dieses Layout bleibt OTA-kompatibel für typische 4-MB-ESP32-Boards, weil zwei App-Partitionen erhalten bleiben. Änderungen an der Partitionstabelle sollten nur erfolgen, wenn die Flash-Größe des Zielboards bekannt ist.
