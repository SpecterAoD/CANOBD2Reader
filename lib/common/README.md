# common

Gemeinsame PlatformIO-Bibliothek fuer Sender und Display.

Enthalten:

- `protocol.h/.cpp`: gemeinsames ESP-NOW-Telemetrieformat und CRC.
- `shared_types.h`: identische Datenstrukturen fuer beide Targets.
- `simulation_data.h`: simulierte Testwerte fuer alle Display-Seiten.
- `PID_Converter.cpp`: OBD2-Rohdatenkonvertierung.
- `CANDecoder.cpp`: einfache CAN-Rohframe-Auswertung fuer die Display-CAN-Seite.

Die Bibliothek wird von beiden Environments automatisch gebaut. Dadurch werden
`src/sender` und `src/display` getrennt kompiliert, teilen sich aber dasselbe
Protokoll.
