# lib/common

Gemeinsame PlatformIO-Bibliothek für Sender, Display und native Tests.

Enthalten:

- `protocol.h/.cpp`: ältere gemeinsame Protokoll-/CRC-Helfer, die während der Migration noch benötigt werden.
- `FirmwareMetadata.h/.cpp`: eingebetteter OTA-Metadatenblock mit Schema, Target, Version und Protokollbereich.
- `shared_types.h`: gemeinsame einfache Datentypen für Sender/Display.
- `simulation_data.h`: gemeinsame Simulationswerte.
- `PID_Converter.cpp`: OBD-II-Rohdatenkonvertierung für Legacy-Pfade.
- `CANDecoder.cpp`: einfache CAN-Rohframe-Auswertung für Diagnose-/Displayansichten.

Neue Telemetrie-Logik liegt primär unter `lib/telemetry/`. Neue Firmware-Validierung sollte über `FirmwareMetadata` und die gemeinsamen Web-Handler laufen, nicht über neue doppelte Parser.
