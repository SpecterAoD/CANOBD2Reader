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

Current intended page set:

1. Main / driving values.
2. Engine values.
3. Consumption / trip.
4. Compact diagnostics.
5. UDS / DTC.
6. CAN sniffer.
7. RPM graph.
8. Additional values.
9. Power management.

Boost as a dedicated page has been removed; MAP/BARO/MAF belong on additional/engine diagnostics pages unless a future design reintroduces a page.

## Rendering rules

- Fresh OK values use normal/green color.
- Warning values use orange.
- Critical values use red.
- Timeout/unknown values use muted gray.
- Display should avoid unnecessary full-screen redraws.

## Navigation

- Short button press: next page.
- Long button press: return to `DisplayConfig::MainPageIndex`.
- Long press must not also trigger short press.

## Target state

Split `DisplayUi.cpp` into:

- pages,
- widgets,
- themes,
- graphs.

The immediate UI priority is removing empty boxes and making simulation pages representative of real telemetry.

