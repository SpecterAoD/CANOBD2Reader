# 23 - Logging

## Contents

- [Overview](#overview)
- [Current state](#current-state)
- [Logging channels](#logging-channels)
- [Persistent diagnostics](#persistent-diagnostics)
- [Log levels](#log-levels)
- [Recommended messages](#recommended-messages)

## Overview

Logging should make vehicle tests diagnosable after the fact, especially when no laptop is connected during driving.

## Current state

`lib/logging/DiagnosticLog.*` provides persistent diagnostic logging. Web status exposes log size and mount state.

## Logging channels

The project uses three different logging paths with different goals:

- **Serial logging (`Logger::*`)**: for interactive development and live debugging over USB serial. Category flags in `LoggingConfig` decide which streams are active.
- **Persistent diagnostic log (`DiagnosticLog::*`)**: for forensic diagnostics on the device filesystem. Use this for significant events and errors that must survive reboots.
- **Telemetry diagnostics (`TraceSenderTelemetry` / `TraceDisplayTelemetry`)**: high-volume transport traces for packet-level troubleshooting. Keep disabled unless actively investigating telemetry.

Performance-critical OBD paths should not emit verbose frame/value logs unless `Obd2DebugEnabled` is explicitly enabled.

## Persistent diagnostics

Requirements:

- log survives until downloaded or rotated,
- log rotation size is configurable,
- log records CAN/OBD/UDS/ESP-NOW/OTA errors,
- empty logs are treated as a bug when diagnostics are enabled.

## Log levels

Recommended levels:

- ERROR: failed operation requiring attention.
- WARN: degraded behavior or timeout.
- INFO: state changes.
- DEBUG: development details.
- TRACE: high-volume protocol details, off by default.

## Recommended messages

Examples:

```text
[SENDER] Auto start enabled
[CAN] Driver started
[OBD] Timeout waiting for ECU response
[UDS] ResponsePending service=0x19
[ESP-NOW] Heartbeat sent seq=123
[DISPLAY] Heartbeat received seq=123
[WebOTA] sha256=...
```
