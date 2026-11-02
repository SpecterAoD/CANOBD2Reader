# Telemetrie-Protokoll (v3)

Sender und Display müssen dieselbe Protokollversion verwenden
(`CANOBD2_PROTOCOL_VERSION` in `platformio.ini`). Ein Display mit anderer
Version verwirft die Pakete und zeigt „Sender-Protokoll passt nicht“.

## Paketrahmen (`lib/telemetry/TelemetryPacket.h`)

| Feld | Typ | Bedeutung |
|---|---|---|
| magic | uint16 | `0xCA02` |
| version | uint8 | Protokollversion (3) |
| type | uint8 | 1 Text, 2 Status, 3 OBD, 4 CAN, 5 Diagnose, **6 LiveSnapshot** |
| sequence | uint32 | fortlaufend; Lückenerkennung auf dem Display |
| timestamp | uint32 | `millis()` des Senders |
| payloadLength | uint16 | ≤ 160 |
| payload | 160 Byte | Inhalt |
| crc | uint16 | CRC-16/Modbus über alles davor |

Die Funkstrecke ist zusätzlich per ESP-NOW-AES (LMK aus den Gerätesecrets)
verschlüsselt, und das Display akzeptiert nur die MAC des Senders.

## LiveSnapshot (Typ 6, `lib/telemetry/LiveData.h`)

```
uint8   metricCount       == 19 (Tabellenversion)
uint8   flags             bit0 = simulierte Werte
uint32  validMask         bit n: Metrik n hat einen Wert
uint32  unsupportedMask   bit n: ECU unterstützt Metrik n nicht
float   values[19]        IEEE-754, little endian
uint8   ageTenths[19]     Alter in 0,1 s; 254 = gesättigt, 255 = fehlend/abgelaufen
```

Metriken (Reihenfolge = Bitnummer): RPM, Speed, CoolantTemp, BatteryVoltage,
EcuVoltage, EngineLoad, IntakeTemp, ManifoldAbsolutePressure,
BarometricPressure, BoostPressureBar, MAF, Throttle, FuelLevel, RunTime,
AmbientTemp, OilTemp, FuelRate, InstantConsumption, AverageConsumption.

Neue Metriken werden **hinten** angefügt; `metricCount` ändert sich, und ältere
Displays verwerfen den Snapshot statt falsch zuzuordnen.

## Textpakete

`TYPE,KEY,NAME,VALUE,UNIT,STATUS,SEQ`, z. B. `STATUS,HEARTBEAT,Heartbeat,42,,OK,1234`.

| KEY | Bedeutung |
|---|---|
| `HEARTBEAT` | Lebenszeichen, 1/s |
| `CAN`, `OBD`, `PID_SUPPORT` | Zustand der Fahrzeugschnittstelle |
| `POWER_STATE`, `ACTIVITY_SCORE`, `POWER_COMMAND` | Power-Manager (`Sleep`/`Wakeup` steuern das Display) |
| `DTC` (Typ DTC) | aktive Fehlercodes, Status `WARN` bei Fehlern |
| `VIN`, `UDS_DID_F190`, `UDS`, `UDS_DTC`, `UDS_NEGATIVE`, `ECU_COUNT` | Diagnose |
| `RAW`, `HINT`, `CAN_*` (Typ CAN) | nur Entwickler-Builds |

## Messwertgültigkeit ab V2.1.1-beta.4

Das Paketformat bleibt v3. Ab 60 Sekunden ohne neue Messung löscht der Sender
beim nächsten Snapshot das jeweilige `validMask`-Bit. Ein weiterlaufender
Funkkontakt verlängert diese Frist nicht. Das kompakte Altersfeld ist bis dahin
auf 25,4 Sekunden begrenzt. Kühlmitteltemperatur und ECU-Spannung haben wie andere
langsame PIDs eine Frischegrenze von 7 Sekunden; schnelle Werte 1,5 Sekunden.

Der Durchschnittsverbrauch umfasst die seit Start (oder Simulationswechsel)
aufgelaufene Kraftstoffmenge und Strecke. Beide Eingaben müssen frisch sein;
Lücken werden nicht integriert. Nullverbrauch zählt mit, Kraftstoff im Stillstand
zur Menge, nicht zur Strecke. Die Anzeige beginnt ab einem gefahrenen Meter.
