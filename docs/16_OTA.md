# 16 - OTA

## Contents

- [Overview](#overview)
- [Current state](#current-state)
- [Partitioning](#partitioning)
- [Firmware metadata](#firmware-metadata)
- [GitHub artifacts](#github-artifacts)
- [Validation rules](#validation-rules)
- [Migration plan](#migration-plan)

## Overview

OTA must work for both sender and display without allowing firmware for one target to be installed on the other.

## Current state

Both environments use `partitions/ota_4mb.csv`. Web-OTA validates filename hints and embedded firmware metadata.

## Partitioning

The OTA partition table must contain at least:

- `ota_0`
- `ota_1`
- enough space for current sender and display firmware images.

## Firmware metadata

Firmware binaries contain a plain ASCII metadata block:

```text
CANOBD2_FW_METADATA_BEGIN;
schema=1;
target=sender;
version=V2.0.0.b6;
protocol=2;
min_protocol=2;
max_protocol=2;
CANOBD2_FW_METADATA_END
```

This block allows browser upload validation without parsing ESP image internals.

Older firmware used a growing `compat_versions` list. That approach was removed because it would become unmaintainable over time. Compatibility is now checked by metadata schema, target, version and protocol range.

## GitHub artifacts

GitHub Actions produce:

- target-specific `.bin`,
- stable `sender.bin` / `display.bin`,
- `.elf`,
- `firmware_manifest.json`,
- `update_manifest.json`,
- `SHA256SUMS.txt`,
- OTA zip,
- flash zip.

`firmware_manifest.json` describes the artifact set. `update_manifest.json` is
the device-facing manifest used by the GitHub update page. It contains separate
`sender` and `display` arrays with version, channel, target, protocol, download
URL and SHA256.

The active device-side manifest URL is:

```text
https://github.com/SpecterAoD/CANOBD2Reader/releases/download/firmware-index/update_manifest.json
```

Release, prerelease and beta workflows merge their generated entry into this
`firmware-index` release. This allows devices to list newer versions and older
rollback candidates from one stable URL.

Sender and display expose the GitHub update UI at:

- `http://<device-ip>/github-update`

The page supports:

- hotspot/WLAN credential entry,
- update channel selection (`stable`, `beta`, `development`),
- manual update check,
- installing the newest compatible forward update,
- listing older compatible versions as rollback candidates.

Rollback is never automatic. It is only started from the web UI and requires a
browser confirmation. The backend also requires an explicit rollback confirmation
flag before installing an older version.

## Validation rules

Web-OTA should reject:

- wrong target,
- missing metadata,
- upload too large for free sketch space,
- aborted writes,
- failed `Update.end(true)`.

It should accept a newer version than the currently installed one.

Rollback through the GitHub update page is allowed only when explicitly confirmed in the browser and when the same metadata checks pass. Rollback is never automatic.

## Migration plan

Future hardening:

1. Add signed firmware metadata.
2. Add signed release manifests.
3. Add manifest signature verification.
