# 19 - GitHub Actions

## Contents

- [Overview](#overview)
- [Workflows](#workflows)
- [Composite action](#composite-action)
- [Artifacts](#artifacts)
- [Versioning](#versioning)

## Overview

GitHub Actions builds firmware, runs tests and creates test/prerelease/release artifacts.

## Workflows

| Workflow | Purpose |
| --- | --- |
| `build.yml` | Normal CI for push, pull request and manual build. |
| `test-build.yml` | Manual test firmware artifact build, no release. |
| `prerelease.yml` | Manual prerelease with Git tag and GitHub prerelease. |
| `release.yml` | Manual release with Git tag and GitHub release. |

## Composite action

`.github/actions/firmware-build/action.yml` contains shared logic:

- setup Python,
- install PlatformIO,
- copy `secrets.example.h` to `secrets.h`,
- validate version and target,
- run `platformio check`,
- build selected target,
- verify OTA metadata in `.bin`,
- optional native tests,
- prepare output artifacts.

## Artifacts

Expected output:

- `CAN_OBD2_sender_V<version>.bin`
- `CAN_OBD2_display_V<version>.bin`
- `sender.bin`
- `display.bin`
- `.elf` files
- `firmware_manifest.json`
- `SHA256SUMS.txt`
- `RELEASE_NOTES.md`
- OTA zip
- flash zip

## Versioning

Manual workflows expect versions without leading `V`; the action produces `V<version>` metadata.

Test builds are marked with:

- `channel: test`
- `is_test_firmware: true`

