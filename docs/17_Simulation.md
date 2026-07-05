# 17 - Simulation

## Contents

- [Overview](#overview)
- [Current state](#current-state)
- [Scenarios](#scenarios)
- [Rules](#rules)
- [Testing](#testing)

## Overview

Simulation allows sender/display development without a vehicle. It must use the same telemetry path as real data.

## Current state

Simulation code lives in `lib/simulation/` and display integration code in `src/display/DisplaySimulation.*`.

## Scenarios

Supported categories:

- normal OBD single-frame values,
- ISO-TP multi-frame VIN/DTC,
- flow control,
- timeout,
- sequence error,
- buffer overflow,
- multiple ECU responses,
- negative response,
- display warning/critical/timeout value sets,
- power states.

## Rules

- Simulation is off by default.
- Simulation state is runtime-only and must not be persisted.
- After restart, simulation is off.
- Simulation must not bypass normal display status handling.

## Testing

Native tests cover simulation contracts and display severity behavior. See [18_Testing.md](18_Testing.md).

