# 04 - Config System

## Contents

- [Overview](#overview)
- [Current state](#current-state)
- [Config modules](#config-modules)
- [Secrets](#secrets)
- [Rules](#rules)
- [Migration plan](#migration-plan)

## Overview

Static configuration lives in `include/config/`. The old monolithic `Config.h` approach has been removed. Code should include only the config module it needs.

## Current state

The project uses separate headers for project, sender, display, network, security, simulation, power, logging, build, OBD and UDS settings.

## Config modules

| Header | Belongs here |
| --- | --- |
| `ProjectConfig.h` | Firmware version, target name, protocol version, telemetry constants. |
| `BuildConfig.h` | Build macro fallbacks and feature flags derived from PlatformIO. |
| `SenderConfig.h` | Sender pins, CAN, OBD/UDS timing, heartbeat and feature flags. |
| `DisplayConfig.h` | Display pins, UI pages, refresh timing, colors and warning thresholds. |
| `NetworkConfig.h` | Wi-Fi AP/STA data, ESP-NOW channel, peer MACs and encryption defaults. |
| `SecurityConfig.h` | Web auth, API token, OTA/restart/simulation protection. |
| `PowerConfig.h` | Start-stop, parked detection, sleep/wakeup and activity scoring. |
| `SimulationConfig.h` | Default simulation state and scenario timing. |
| `LoggingConfig.h` | Serial and persistent diagnostic log settings. |
| `ObdConfig.h` | Requested PIDs, fallback barometric pressure and scan intervals. |
| `UdsConfig.h` | ECU IDs, allowed services, DID candidates and backoff timing. |

## Secrets

Use:

- `include/secrets.example.h` for documented placeholder values and CI.
- `include/secrets.h` for local credentials.

`include/secrets.h` must not be committed.

## Rules

- Config files must not contain mutable runtime state.
- Runtime switches such as “simulation enabled now” belong in runtime state.
- Secrets must be optional at compile time but clearly warned in web status.
- Avoid duplicate thresholds in UI and simulation; simulation should reuse the same config.

## Migration plan

1. Keep adding settings only to the narrowest config header.
2. Remove remaining legacy aliases when encountered.
3. Add tests for new config invariants under `test/test_config*`.

