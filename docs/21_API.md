# 21 - API

## Contents

- [Overview](#overview)
- [Common endpoints](#common-endpoints)
- [Sender endpoints](#sender-endpoints)
- [Capability endpoints](#capability-endpoints)
- [Example status JSON](#example-status-json)

## Overview

Web APIs are JSON-based unless serving HTML or firmware upload responses.

## Common endpoints

| Endpoint | Method | Purpose |
| --- | --- | --- |
| `/` | GET | Web UI. |
| `/status` | GET | Device status JSON. |
| `/api/restart` | POST | Restart device after response. |
| `/api/simulation` | GET | Simulation state. |
| `/api/simulation/on` | POST | Enable runtime simulation. |
| `/api/simulation/off` | POST | Disable runtime simulation. |
| `/api/simulation/toggle` | POST | Toggle runtime simulation. |
| `/api/simulation/scenario` | GET/POST | Get or set scenario. |
| OTA upload endpoint | POST | Firmware upload. |

## Sender endpoints

Expected sender-specific APIs:

- diagnostics snapshot,
- diagnostic log download,
- capability scan start/stop,
- CAN sniffer start/stop/baseline/export,
- LED test where enabled.

## Capability endpoints

Target behavior:

| Endpoint | Purpose |
| --- | --- |
| `/api/capabilities/obd/start` | Start OBD PID scan. |
| `/api/capabilities/uds/start` | Start UDS scan. |
| `/api/capabilities/can/start` | Start passive CAN sniffer. |
| `/api/capabilities/stop` | Stop active scan. |
| `/api/capabilities/export.json` | Export all scan results. |

Exact endpoint names should be kept in sync with `WebConsoleHandler.cpp`.

## Example status JSON

```json
{
  "firmware": "V2.0.0.b4",
  "target": "sender",
  "protocol": 2,
  "buildTime": "Jul 5 2026 08:46:54",
  "otaStatus": "idle",
  "otaMetadata": "CANOBD2_FW_METADATA_BEGIN;target=sender;...",
  "freeSketchSpace": 1900544,
  "sketchSize": 1050821,
  "flashSize": 4194304
}
```

