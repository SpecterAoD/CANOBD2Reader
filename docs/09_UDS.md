# 09 - UDS

## Contents

- [Overview](#overview)
- [Safety model](#safety-model)
- [Current state](#current-state)
- [Supported services](#supported-services)
- [VW/MQB discovery](#vwmqb-discovery)
- [Response pending](#response-pending)
- [Target state](#target-state)

## Overview

UDS, Unified Diagnostic Services, is used for read-only extended diagnostics such as VIN, ECU identifiers, DTC information and future vehicle-specific values.

## Safety model

The project must stay read-only by default. UDS is used to request information, not to change coding, adaptations or actuator states.

## Current state

`lib/uds/` contains a UDS client, decoder and diagnostic state. `include/config/UdsConfig.h` defines allowed services and candidate IDs/DIDs.

## Supported services

| Service | Name | Intended use |
| --- | --- | --- |
| `0x10` | DiagnosticSessionControl | Optional session setup when needed. |
| `0x22` | ReadDataByIdentifier | Read VIN and ECU identifiers. |
| `0x19` | ReadDTCInformation | Read DTC information conservatively. |
| `0x3E` | TesterPresent | Keep a diagnostic session alive when explicitly active. |

Unsafe write/control services must stay blocked unless a future design review explicitly allows a narrow use case.

## VW/MQB discovery

Configured request IDs should include common diagnostic targets such as:

- ECU: request `0x7E0`, response `0x7E8`
- Transmission: request `0x7E1`, response `0x7E9`
- ABS/ESP: request `0x7E2`, response `0x7EA`
- BCM: request `0x7E4`, response `0x7EC`
- Instrument cluster: request `0x714`, response `0x77E`

Candidate DIDs should be treated as discovery hints, not guaranteed universal truth.

## Response pending

Negative response `0x7F <service> 0x78` means **ResponsePending**. It is not a hard failure.

Behavior:

- increment pending counter,
- keep waiting up to a configured total timeout,
- apply backoff if pending repeats too often,
- expose status in web and display diagnostics.

## Target state

- UDS scanner is manually started from the web UI.
- OBD live values keep priority over UDS.
- Results are exportable as JSON.
- Vehicle profiles can tune allowed ECU/DID lists.

