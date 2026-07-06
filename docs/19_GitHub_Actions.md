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
| `beta-release.yml` | Manual beta firmware as GitHub prerelease. |
| `prerelease.yml` | Manual prerelease with Git tag and GitHub prerelease. |
| `release.yml` | Manual release with Git tag and GitHub release. |
| `docs.yml` | Manual/CI documentation generation and documentation artifact upload. |
| `size-report.yml` | Manual firmware size report for sender/display and OTA capacity checks. |

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
- `update_manifest.json`
- `SHA256SUMS.txt`
- `RELEASE_NOTES.md`
- OTA zip
- flash zip

`update_manifest.json` is merged with the existing `firmware-index` release and
uploaded back to:

```text
https://github.com/SpecterAoD/CANOBD2Reader/releases/download/firmware-index/update_manifest.json
```

Devices use this stable URL for GitHub update checks, channel filtering and
manual rollback selection.

## Versioning

Manual workflows expect versions without leading `V`; the action produces `V<version>` metadata.

Accepted examples include `2.0.0.b6`, `2.0.0-beta.1` and `2.0.0-rc1`. Workflows normalize the leading `V` to avoid tags such as `VV2.0.0...`.

Test builds are marked with:

- `channel: development`
- `is_test_firmware: true`

Test builds are uploaded only as workflow artifacts. They do not update the
device-facing `firmware-index` release.
