# 25 - Roadmap

## Contents

- [Short-term](#short-term)
- [Mid-term](#mid-term)
- [Long-term](#long-term)
- [Known limitations](#known-limitations)
- [Issue-ready backlog](#issue-ready-backlog)

## Short-term

1. Real-test Web-OTA and GitHub rollback from older firmware to `V2.0.0.b6`.
2. Repair/verify web buttons for OBD PID scan, UDS scan and CAN sniffer on real hardware.
3. Validate live OBD speed after the fast/slow PID scheduler change.
4. Validate persistent diagnostic logs in vehicle tests.
5. Improve raw CAN/HEX diagnostic visibility on the display and in exports.
6. Define and implement SavvyCAN dedicated sender/display mode for live USB logging.

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
6. Optional SavvyCAN/GVRET-compatible RX-only USB bridge mode.

## Known limitations

- UDS DIDs are partly vehicle-specific and must be discovered carefully.
- CAN sniffer candidates need human validation.
- Web security is local/basic and should not be exposed to untrusted networks.
- Display layout still needs ergonomic refinement, especially the diagnostic pages.
- UDS and CAN signal discovery should remain manual/read-only until vehicle-specific behavior is better understood.

## Issue-ready backlog

The detailed audit backlog is maintained in [28_Audit_Work_Items.md](28_Audit_Work_Items.md). Each item is written so it can be copied into a GitHub issue with priority, scope and acceptance criteria.
