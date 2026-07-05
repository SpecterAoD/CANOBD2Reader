# 23 - Logging

## Contents

- [Overview](#overview)
- [Current state](#current-state)
- [Persistent diagnostics](#persistent-diagnostics)
- [Log levels](#log-levels)
- [Recommended messages](#recommended-messages)

## Overview

Logging should make vehicle tests diagnosable after the fact, especially when no laptop is connected during driving.

## Current state

`lib/logging/DiagnosticLog.*` provides persistent diagnostic logging. Web status exposes log size and mount state.

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

