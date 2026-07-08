# 12 - Display

## Contents

- [Overview](#overview)
- [Current state](#current-state)
- [Pages](#pages)
- [Rendering rules](#rendering-rules)
- [Navigation](#navigation)
- [Target state](#target-state)

## Overview

The display firmware receives telemetry and renders a compact, readable dashboard for driving and diagnostics.

## Current state

Display code lives in `src/display/`. Shared severity logic lives in `lib/display/`.

## Pages

Current intended page set is split into a normal driving mode and a diagnostic
mode. The diagnostic mode is runtime-only and is reset on reboot.

Normal driving mode:

1. Main / driving values.
2. Engine values.
3. Consumption / trip.
4. Compact diagnostics.

Diagnostic mode additionally exposes:

5. UDS / DTC.
6. CAN sniffer.
7. Additional values, including ECU voltage from OBD PID `0x42`.
8. Power management.

The dedicated boost page and the dedicated RPM graph page have been removed. RPM remains on the main page. MAP, BARO and MAF belong on the additional values page or engine diagnostics. This keeps the page list compact and leaves more room for useful diagnostic/CAN information.

## Rendering rules

- Fresh OK values use normal/green color.
- Warning values use orange.
- Critical values use red.
- Timeout/unknown values use muted gray.
- Display should avoid unnecessary full-screen redraws.
- Empty values must not be drawn as green “OK” fields. If no value is available, the field should show a meaningful fallback such as `N/A`, `--`, a raw CAN/HEX value, or be suppressed.
- Main live values should use the latest telemetry directly; smoothing must never hide a real timeout.

## Navigation

- Short button press: next page.
- Long button press: return to `DisplayConfig::MainPageIndex`.
- Very long button press: toggle diagnostic pages until the next reboot.
- Display web UI: `Diagnose-Seiten umschalten` toggles the same runtime state.
- Long press must not also trigger short press.

## Target state

Split `DisplayUi.cpp` into:

- pages,
- widgets,
- themes,
- graphs.

The immediate UI priority is removing empty boxes and making simulation pages representative of real telemetry.
