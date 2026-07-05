# 26 - Developer Guide

## Contents

- [Setup](#setup)
- [Build](#build)
- [Tests](#tests)
- [Simulation](#simulation)
- [OTA testing](#ota-testing)
- [Release workflow](#release-workflow)
- [Documentation workflow](#documentation-workflow)

## Setup

```bash
git clone https://github.com/SpecterAoD/CANOBD2Reader.git
cd CANOBD2Reader
python -m pip install platformio
```

Create local secrets if needed:

```bash
cp include/secrets.example.h include/secrets.h
```

## Build

```bash
pio run -e sender
pio run -e display
```

## Tests

```bash
pio test -e native
```

## Simulation

Simulation is off by default. Enable it from the web UI or test code only for runtime testing. It must not be persisted across restart.

## OTA testing

1. Build firmware.
2. Open sender/display web UI.
3. Upload `sender.bin` only to sender and `display.bin` only to display.
4. Confirm `/status` shows the new firmware version.
5. Confirm wrong target files are rejected.

## Release workflow

Use GitHub Actions:

- `Test Firmware Build` for manual test artifacts.
- `PreRelease Firmware` for beta/rc artifacts.
- `Release Firmware` for stable releases.

Enter versions without leading `V`.

## Documentation workflow

When code changes behavior, update the matching chapter under `docs/`. Also update [25_Roadmap.md](25_Roadmap.md) when a known limitation is fixed or a new risk appears.

