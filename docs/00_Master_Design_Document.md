# 00 - Master Design Document

## Contents

- [Project vision](#project-vision)
- [Design goals](#design-goals)
- [Architecture principles](#architecture-principles)
- [High-level architecture](#high-level-architecture)
- [Project structure](#project-structure)
- [Development strategy](#development-strategy)
- [Definition of done](#definition-of-done)
- [References](#references)

## Project vision

CANOBD2Reader is a two-device vehicle dashboard and diagnostics platform:

- **Sender**: ESP32 DevKit V1 connected to the vehicle CAN/OBD-II interface.
- **Display**: LilyGO T-Display S3 receiving telemetry and rendering a driver-readable dashboard.

The project should provide reliable live values, safe diagnostics, OTA updates, simulation support and a path toward vehicle-specific read-only discovery for values that are not available through standard OBD-II.

## Design goals

- Keep sender and display as separate firmware targets in one PlatformIO project.
- Prefer typed protocols over free-form strings.
- Keep read-only diagnostics safe by default.
- Keep OTA functional at every step.
- Make simulation good enough to test display and sender/display contracts without a vehicle.
- Make future VW MQB/MQB Evo extensions possible without hard-coding unsafe behavior.

## Architecture principles

- **Modularity**: CAN, ISO-TP, OBD, UDS, telemetry, web, power and display code are separate modules.
- **Maintainability**: small classes, clear ownership and documented responsibilities.
- **Extensibility**: new PIDs, DIDs, display pages and web APIs should be additive.
- **Testability**: protocol logic should compile under the `native` test environment.
- **OTA compatibility**: partitioning, firmware metadata and target validation are mandatory.
- **Platform isolation**: hardware-specific code stays behind Arduino/ESP32 boundaries.

## High-level architecture

```mermaid
flowchart TD
    Vehicle["Vehicle CAN / OBD-II"] --> CAN["TWAI / CAN Driver"]
    CAN --> Router["CAN Router"]
    Router --> ISOTP["ISO-TP"]
    ISOTP --> OBD["OBD-II Client"]
    ISOTP --> UDS["UDS Client"]
    OBD --> Runtime["Sender Runtime State"]
    UDS --> Runtime
    Runtime --> Telemetry["Telemetry Codec"]
    Telemetry --> EspNow["ESP-NOW Transport"]
    EspNow --> DisplayRx["Display Receiver"]
    DisplayRx --> DisplayRuntime["Display Runtime State"]
    DisplayRuntime --> UI["Display UI Pages"]
    Web["Sender Web Console"] --> Runtime
    Simulation["Simulation"] --> Runtime
```

## Project structure

```text
CANOBD2Reader/
├── platformio.ini
├── VERSION.txt
├── include/
│   ├── common_config.h
│   ├── secrets.example.h
│   └── config/
├── lib/
│   ├── can_router/
│   ├── capabilities/
│   ├── common/
│   ├── display/
│   ├── isotp/
│   ├── logging/
│   ├── obd/
│   ├── power/
│   ├── runtime/
│   ├── simulation/
│   ├── status/
│   ├── telemetry/
│   ├── transport/
│   ├── uds/
│   └── web/
├── src/
│   ├── sender/
│   └── display/
├── test/
├── docs/
└── .github/
```

For details see [02_Project_Structure.md](02_Project_Structure.md).

## Development strategy

### Phase 1 - Stabilize

- Keep sender auto-starting without web interaction.
- Keep heartbeats independent from OBD success.
- Keep OTA metadata and target checks reliable.
- Keep native tests green.

### Phase 2 - Make diagnostics useful

- Finish OBD capability scanner UI.
- Finish UDS capability scanner and read-only DID workflow.
- Improve CAN sniffer workflow with baseline/diff/export.

### Phase 3 - Improve user experience

- Refine display page layouts.
- Improve refresh rates and page-specific value priorities.
- Make web console actions visible and debuggable.

### Phase 4 - Vehicle-specific profiles

- Add profile-based defaults for VW MQB/MQB Evo.
- Keep profiles optional and read-only.
- Store discovered capabilities safely.

## Definition of done

A change is done when:

- `pio run -e sender` succeeds.
- `pio run -e display` succeeds.
- `pio test -e native` succeeds for portable logic changes.
- OTA compatibility is not broken.
- Sender/display telemetry compatibility is preserved.
- Documentation is updated when behavior or architecture changes.
- Remaining hardware validation is explicitly listed.

## References

- [Architecture](01_Architecture.md)
- [OTA](16_OTA.md)
- [Testing](18_Testing.md)
- [Roadmap](25_Roadmap.md)

