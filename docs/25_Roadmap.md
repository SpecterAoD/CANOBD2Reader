# 25 - Roadmap

## Contents

- [Short-term](#short-term)
- [Mid-term](#mid-term)
- [Long-term](#long-term)
- [Known limitations](#known-limitations)

## Short-term

1. Real-test Web-OTA from older firmware to `V2.0.0.b4`.
2. Repair/verify web buttons for OBD PID scan, UDS scan and CAN sniffer.
3. Improve display refresh rate and remove empty UI boxes.
4. Validate persistent diagnostic logs in vehicle tests.

## Mid-term

1. Finish capability scanner result tables and JSON export.
2. Improve UDS response pending/backoff visualization.
3. Split display UI into pages/widgets/themes.
4. Split shared web HTML/JS into reusable assets.
5. Add vehicle profile defaults for VW MQB/MQB Evo.

## Long-term

1. Signed OTA metadata.
2. Optional persisted capability profile.
3. Rich CAN signal discovery workflow.
4. Better trip statistics and historical graphs.
5. Generated Doxygen documentation.

## Known limitations

- UDS DIDs are partly vehicle-specific and must be discovered carefully.
- CAN sniffer candidates need human validation.
- Web security is local/basic and should not be exposed to untrusted networks.
- Display layout still needs ergonomic refinement.

