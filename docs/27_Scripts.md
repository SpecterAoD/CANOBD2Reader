# 27 - Scripts

## Contents

- [Overview](#overview)
- [Build and checks](#build-and-checks)
- [Flashing and monitoring](#flashing-and-monitoring)
- [OTA helpers](#ota-helpers)
- [Documentation generators](#documentation-generators)
- [Safety and release helpers](#safety-and-release-helpers)
- [Maintenance rules](#maintenance-rules)

## Overview

The `scripts/` folder contains local developer tooling. PowerShell scripts are intended mainly for Windows and VS Code. Python scripts are kept dependency-light and should work on Windows, Linux and GitHub Actions.

All scripts should be run from the repository root unless the script itself says otherwise.

## Build and checks

| Script | Purpose |
| --- | --- |
| `dev_check.ps1` | Runs the normal local quality pass: sender build, display build, native tests and PlatformIO checks. |
| `clean_build.ps1` | Removes/rebuilds PlatformIO output and creates fresh sender/display firmware. |
| `doctor.ps1` | Checks local tools such as Git, Python, PlatformIO, esptool and serial dependencies. |
| `ensure_intelhex.py` | Ensures the Python dependency needed by esptool/PlatformIO bootloader generation is present. |
| `check_line_endings.py` | Detects CRLF/LF line-ending problems before they become noisy diffs. |
| `check_protocol_version.py` | Compares protocol version declarations in code/metadata related files. |

## Flashing and monitoring

| Script | Purpose |
| --- | --- |
| `list_ports.ps1` | Lists local serial ports to help identify sender/display COM ports. |
| `flash_sender.ps1` | Convenience wrapper for flashing the sender environment. |
| `flash_display.ps1` | Convenience wrapper for flashing the display environment. |
| `monitor_sender.ps1` | Opens a serial monitor for the sender. |
| `monitor_display.ps1` | Opens a serial monitor for the display. |
| `upload_both.ps1` | Convenience upload path for both devices where supported. |
| `auto_upload_port.py` | Helper for automatic upload port selection. |

## OTA helpers

| Script | Purpose |
| --- | --- |
| `ota_upload_sender.ps1` | Uploads sender firmware to a sender device over OTA/IP. |
| `ota_upload_display.ps1` | Uploads display firmware to a display device over OTA/IP. |
| `decode_status_json.py` | Reads/formats `/status` JSON for diagnostics. |
| `validate_manifest.py` | Validates firmware manifests against files and SHA256 values. |

## Documentation generators

| Script | Output |
| --- | --- |
| `generate_project_index.py` | `docs/AUTO_PROJECT_INDEX.md` |
| `generate_build_docs.py` | `docs/AUTO_BUILD_FLAGS.md` |
| `generate_config_reference.py` | `docs/AUTO_CONFIG_REFERENCE.md` |
| `generate_test_overview.py` | `docs/AUTO_TEST_OVERVIEW.md` |
| `export_firmware_info.py` | Firmware artifact/version/size/SHA256 information. |
| `compare_firmware_size.py` | Firmware size comparison and OTA size report. |
| `verify_docs.py` | Checks important documentation files and internal Markdown links. |

Generated files must clearly remain marked as generated. Human-maintained design decisions belong in the numbered docs.

## Safety and release helpers

| Script | Purpose |
| --- | --- |
| `check_secrets.py` | Fails when real secrets, private keys or `include/secrets.h` are accidentally committed. |
| `backup_config.ps1` | Backs up local private config/secrets outside the repository. |
| `make_release_notes.py` | Creates release notes from version, target, channel and optional Git commits. |
| `prepare_beta.ps1` | Helps prepare a local beta version. |
| `apply_firmware_version.py` | PlatformIO pre-script that injects `VERSION.txt` into build flags. |
| `repo_utils.py` | Shared Python helpers for scripts. |

## Maintenance rules

- Do not place real credentials in scripts.
- Scripts must use paths relative to the repository root.
- CI uses `include/secrets.example.h`; local private `include/secrets.h` must remain ignored.
- When adding a script, document it here and update generated docs if relevant.
