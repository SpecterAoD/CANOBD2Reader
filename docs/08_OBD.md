# 08 - OBD-II

## Contents

- [Overview](#overview)
- [Current state](#current-state)
- [Supported values](#supported-values)
- [Capability scanner](#capability-scanner)
- [Derived values](#derived-values)
- [Target state](#target-state)

## Overview

OBD-II Mode 01 provides standardized live data. It is the safest primary source for common dashboard values.

## Current state

The project contains:

- `lib/obd/PidDecoder.*`
- `lib/obd/BoostCalculator.*`
- `lib/obd/DtcDecoder.h`
- `lib/obd/VinDecoder.h`
- sender OBD scheduler and diagnostics.

Live polling is split into two configured groups:

- `ObdConfig::FastLivePids`: speed, RPM, coolant temperature and ECU voltage.
- `ObdConfig::SlowLivePids`: slower values such as oil temperature, engine load, intake temperature, MAF, MAP, BARO, fuel level and runtime.

`SenderConfig::FastLiveObdPollIntervalMs` is tuned for responsive display values, while `SenderConfig::SlowLiveObdPollIntervalMs` avoids starving the bus with low-priority requests. Each scheduler tick sends at most one PID request so CAN/ISO-TP and heartbeat work can continue.

## Supported values

Important PIDs:

| PID | Value |
| --- | --- |
| `0x04` | Calculated engine load |
| `0x05` | Coolant temperature |
| `0x0B` | MAP / intake manifold absolute pressure |
| `0x0C` | RPM |
| `0x0D` | Speed |
| `0x0F` | Intake air temperature |
| `0x10` | MAF |
| `0x11` | Throttle position |
| `0x1F` | Runtime since engine start |
| `0x2F` | Fuel tank level |
| `0x33` | Barometric pressure |
| `0x42` | Control module voltage |
| `0x46` | Ambient air temperature |
| `0x5C` | Oil temperature |
| `0x5E` | Fuel rate |

## Capability scanner

The scanner should:

1. Query supported PID masks.
2. Decode supported PIDs.
3. Probe supported PIDs once.
4. Record status as OK, unsupported, timeout or decode error.
5. Let the user choose future runtime PIDs without storing unsafe defaults yet.

## Derived values

Boost pressure must be calculated centrally in sender-side logic:

```cpp
boostPressureBar = (mapKpa - barometricPressureKpa) / 100.0f;
```

If PID `0x33` is unsupported, use `ObdConfig::DefaultBarometricPressureKpa`.

The display must show already calculated values; it should not duplicate boost calculations.

## Target state

- Live-value PID priorities are configurable.
- Unsupported PIDs are not aggressively polled.
- Capability scan results can later be persisted safely.
- The web capability scan should feed the runtime PID selection without changing compile-time defaults.
