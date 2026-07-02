# Architecture

```mermaid
flowchart LR
    Vehicle["Vehicle CAN bus"] --> TWAI["ESP32 TWAI driver"]
    TWAI --> ISO["ISO-TP handler"]
    ISO --> OBD["OBD service / PID decoder"]
    OBD --> Derived["Derived values\nBoost, consumption"]
    Derived --> Telemetry["Telemetry packet\nMagic, version, sequence, CRC"]
    Telemetry --> EspNow["ESP-NOW transport"]
    EspNow --> Queue["Display receive queue"]
    Queue --> Decode["Telemetry decode + validation"]
    Decode --> State["Display runtime data"]
    State --> UI["Dashboard pages"]
```

## Firmware split

- `env:sender`: ESP32 DevKit V1, CAN/OBD reader and telemetry sender.
- `env:display`: LilyGO T-Display S3, ESP-NOW receiver, dashboard and web OTA.
- `env:native`: host-side unit tests for protocol, CRC, ISO-TP, OBD, simulation
  and security helpers.

## Shared modules

- `lib/telemetry`: binary packet envelope and CRC validation.
- `lib/isotp`: ISO-TP request/response and reassembly.
- `lib/can_router`: fixed-size CAN frame fan-out for future TWAI routing.
- `lib/obd`: PID decoding and derived OBD calculations such as boost pressure.
- `lib/simulation`: runtime-only OBD/ISO-TP/display simulation scenarios.
- `lib/runtime`: mutable runtime state such as sender loop state and
  consumption averaging.
- `lib/display`: display severity calculation.
- `lib/web`: shared web authentication helpers.

## Configuration model

Compile-time configuration is exposed through `include/Config.h` as a
compatibility facade. New static values should be added to focused config files
such as `ProjectConfig.h`, `SenderConfig.h`, `DisplayConfig.h`,
`SimulationConfig.h` or `SecurityConfig.h`.

Runtime values must not be added to config headers. Mutable state belongs in
`lib/runtime` or in the relevant firmware module.

## CAN routing status

`lib/can_router` provides the shared listener model requested for avoiding
competing TWAI reads. It is covered by native tests. The current live sender path
still uses the existing CAN/OBD flow; wiring TWAI RX through `CanRouter` should
be done as a dedicated hardware-tested step so ISO-TP timing is not changed
silently.
