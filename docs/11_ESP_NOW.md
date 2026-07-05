# 11 - ESP-NOW

## Contents

- [Overview](#overview)
- [Current state](#current-state)
- [Channel and peer configuration](#channel-and-peer-configuration)
- [Status handling](#status-handling)
- [Recovery](#recovery)

## Overview

ESP-NOW transports telemetry from sender to display. It must remain independent from OBD success.

## Current state

Transport code lives in `lib/transport/EspNowTelemetryTransport.*` and sender/display integration code. Network constants are configured through `NetworkConfig`.

## Channel and peer configuration

Both devices must use the same ESP-NOW channel. Secrets and peer MACs belong in `include/secrets.h` with defaults from `include/secrets.example.h`.

## Status handling

Display status rules:

- ESP-NOW OK: any valid heartbeat or telemetry packet arrives before timeout.
- CAN status: reported by sender.
- OBD status: reported by sender.
- Simulation status: runtime flag from sender/display simulation path.

## Recovery

Expected behavior:

1. Sender sends heartbeat at a fixed interval.
2. Display marks ESP-NOW stale after `DisplayConfig::EspNowTimeoutMs`.
3. Receiving any valid packet restores ESP-NOW OK.
4. OBD timeout must not mark ESP-NOW as failed.

