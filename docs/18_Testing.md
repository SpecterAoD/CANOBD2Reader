# 18 - Testing

## Contents

- [Overview](#overview)
- [Native tests](#native-tests)
- [Build tests](#build-tests)
- [Hardware tests](#hardware-tests)
- [Required commands](#required-commands)

## Overview

Testing combines native unit tests, PlatformIO firmware builds and manual hardware validation.

## Native tests

Existing native test areas include:

- activity monitor,
- boost calculation,
- CAN router,
- CAN signal diff,
- config,
- CRC,
- diagnostic log,
- display severity/status,
- heartbeat,
- ISO-TP,
- LED controller,
- OBD capability and diagnostics,
- PID decoder,
- runtime,
- security,
- sender/display simulation contract,
- telemetry codec,
- UDS and response pending.

## Build tests

Build both firmware targets:

```bash
pio run -e sender
pio run -e display
```

## Hardware tests

Manual tests must validate:

- CAN wiring and TWAI startup,
- OBD responses in vehicle,
- ESP-NOW range,
- display page navigation,
- Web-OTA sender/display,
- capability scanner buttons,
- CAN sniffer baseline/diff workflow.

## Required commands

Before merging protocol or architecture changes:

```bash
pio run -e sender
pio run -e display
pio test -e native
```

