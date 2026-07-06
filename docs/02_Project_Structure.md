# 02 - Project Structure

## Contents

- [Overview](#overview)
- [Root files](#root-files)
- [include](#include)
- [lib](#lib)
- [src](#src)
- [test](#test)
- [docs](#docs)
- [Migration notes](#migration-notes)

## Overview

The repository is one PlatformIO project with multiple environments. Code is organized by responsibility, not by Arduino sketch folders.

## Root files

| Path | Purpose |
| --- | --- |
| `platformio.ini` | Build environments for `sender`, `display` and `native`. |
| `VERSION.txt` | Source version used by `scripts/apply_firmware_version.py`. |
| `partitions/ota_4mb.csv` | OTA-compatible partition layout. |
| `scripts/` | Helper scripts for version injection and upload automation. |
| `.github/` | CI, test-build, prerelease and release workflows. |

## include

`include/config/` contains static compile-time configuration:

- `ProjectConfig.h`
- `BuildConfig.h`
- `SenderConfig.h`
- `DisplayConfig.h`
- `NetworkConfig.h`
- `SecurityConfig.h`
- `PowerConfig.h`
- `SimulationConfig.h`
- `LoggingConfig.h`
- `ObdConfig.h`
- `UdsConfig.h`
- `UpdateConfig.h`

Secrets are intentionally split:

- `include/secrets.example.h`: committed defaults for CI and examples.
- `include/secrets.h`: local private overrides, ignored by Git.

## lib

| Folder | Responsibility |
| --- | --- |
| `lib/can_router` | CAN frame dispatch and listener fan-out. |
| `lib/capabilities` | OBD PID, UDS ECU/DID and CAN signal discovery structures. |
| `lib/common` | Shared legacy-compatible helpers, firmware metadata and protocol utilities. |
| `lib/display` | Display severity and shared display logic. |
| `lib/isotp` | ISO-TP handler and reassembler. |
| `lib/logging` | Persistent diagnostic log helpers. |
| `lib/obd` | PID decoder, boost calculation, VIN/DTC helpers. |
| `lib/power` | Activity monitor and vehicle state evaluation. |
| `lib/runtime` | Sender/display/web runtime state. |
| `lib/simulation` | OBD, ISO-TP and display simulation scenarios. |
| `lib/status` | LED/status logic. |
| `lib/telemetry` | Packet, codec and sequence helpers. |
| `lib/transport` | ESP-NOW telemetry transport. |
| `lib/uds` | UDS client, decoder and diagnostics. |
| `lib/update` | GitHub manifest parsing, channel/version selection and OTA update decisions. |
| `lib/web` | Shared web, authentication, OTA and status helpers. |

## src

`src/sender/` contains sender firmware orchestration:

- `main.cpp`
- `SenderApp.*`
- OBD, UDS, heartbeat, telemetry, ESP-NOW, power and capability scheduler files.
- `WebConsoleHandler.*`

`src/display/` contains display firmware orchestration:

- `main.cpp`
- `DisplayApp.*`
- `DisplayReceiver.*`
- `DisplayData.*`
- `DisplayUi.*`
- `DisplayOta.*`
- `DisplaySimulation.*`

## test

Native Unity tests cover portable logic. Hardware-dependent code is kept behind interfaces or tested through pure helper classes where possible.

## docs

`docs/` contains this documentation. `docs/archive/` stores legacy snippets and migration references.

## scripts

`scripts/` contains local developer tooling. PowerShell scripts are optimized for Windows/VS Code development; Python scripts are cross-platform checks and documentation generators. See [27_Scripts.md](27_Scripts.md) for the maintained script reference.

Important groups:

- build/check: `dev_check.ps1`, `clean_build.ps1`, `doctor.ps1`
- flashing/monitoring: `flash_sender.ps1`, `flash_display.ps1`, `monitor_sender.ps1`, `monitor_display.ps1`
- OTA helpers: `ota_upload_sender.ps1`, `ota_upload_display.ps1`
- docs: `generate_project_index.py`, `generate_build_docs.py`, `generate_config_reference.py`, `generate_test_overview.py`
- safety/release: `check_secrets.py`, `validate_manifest.py`, `make_release_notes.py`

## Migration notes

Current large files that should be split further:

- `src/display/DisplayUi.cpp`: target split into pages/widgets/themes.
- `src/sender/WebConsoleHandler.cpp`: target split into smaller shared web assets and endpoint handlers.
