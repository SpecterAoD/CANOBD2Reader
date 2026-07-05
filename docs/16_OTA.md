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
target=sender;
version=V2.0.0.b4;
protocol=2;
compat_versions=...;
CANOBD2_FW_METADATA_END
```

This block allows browser upload validation without parsing ESP image internals.

## GitHub artifacts

GitHub Actions produce:

- target-specific `.bin`,
- stable `sender.bin` / `display.bin`,
- `.elf`,
- `firmware_manifest.json`,
- `SHA256SUMS.txt`,
- OTA zip,
- flash zip.

## Validation rules

Web-OTA should reject:

- wrong target,
- missing metadata,
- upload too large for free sketch space,
- aborted writes,
- failed `Update.end(true)`.

It should accept a newer version than the currently installed one.

## Migration plan

Future hardening:

1. Add signed firmware metadata.
2. Validate SHA256 against a manifest where possible.
3. Add explicit stable/beta/test channel display in web status.

