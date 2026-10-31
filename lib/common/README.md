# lib/common

Gemeinsame PlatformIO-Bibliothek für Sender, Display und native Tests.

Enthalten:

- `protocol.h/.cpp`: Aufbau der Status-Textpakete.
- `FirmwareMetadata.h/.cpp`: eingebetteter OTA-Metadatenblock mit Schema, Target, Version und Protokollbereich.
- `shared_types.h`: gemeinsame einfache Datentypen für Sender/Display.
- `simulation_data.h`: gemeinsame Simulationswerte.
- `PID_Converter.cpp`: OBD-II-Rohdatenkonvertierung für Legacy-Pfade.
- `CANDecoder.cpp`: einfache CAN-Rohframe-Auswertung für Diagnose-/Displayansichten.

Neue Telemetrie-Logik liegt unter `lib/telemetry/` (Livewerte: `LiveData`). Firmware-Validierung: `lib/update/OtaImageValidator` liest den Block aus `FirmwareMetadata`; jedes Image muss ihn enthalten (CI prüft das mit `scripts/check_firmware_image.py`).
