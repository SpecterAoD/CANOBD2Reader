# 03 - Coding Guidelines

## Contents

- [Language and build](#language-and-build)
- [Design principles](#design-principles)
- [Naming](#naming)
- [Configuration](#configuration)
- [State ownership](#state-ownership)
- [Comments and Doxygen](#comments-and-doxygen)
- [Error handling](#error-handling)
- [Testing expectations](#testing-expectations)

## Language and build

- Use C++17.
- Prefer `constexpr` for constants.
- Prefer `enum class` over unscoped enums.
- Avoid dynamic allocation in protocol paths.
- Keep Arduino/ESP32-specific code out of native-testable core logic.

## Design principles

- **SOLID**: one responsibility per class/module.
- **DRY**: no duplicate parser, CRC, decoder or config implementations.
- **KISS**: prefer explicit small helpers over clever generic code.
- **RAII** where supported and safe for embedded usage.
- **Fail safe**: diagnostics should be read-only unless explicitly designed otherwise.

## Naming

| Item | Convention | Example |
| --- | --- | --- |
| Classes | `PascalCase` | `IsoTpHandler` |
| Methods | `camelCase` | `receiveResponse()` |
| Variables | `camelCase` | `lastHeartbeatMs` |
| Constants | `PascalCase` in config namespaces | `SenderConfig::HeartbeatIntervalMs` |
| Namespaces | `PascalCase` for config, descriptive for modules | `ProjectConfig`, `Telemetry` |
| Files | Class or module name | `TelemetryCodec.cpp` |

## Configuration

Static settings belong in `include/config/`. Runtime state must not be stored in config headers.

## State ownership

Mutable state belongs in:

- `lib/runtime`
- sender/display app state objects
- explicit scheduler objects

Avoid hidden global variables. When a singleton-style object is unavoidable on Arduino, keep the mutable state narrow and documented.

## Comments and Doxygen

Public classes and public functions should have Doxygen comments explaining:

- responsibility,
- inputs and outputs,
- timing/side effects,
- safety assumptions.

Prefer comments that explain **why** code exists. Do not repeat what the next line already says.

Example:

```cpp
/**
 * Reassembles ISO-TP segmented payloads from one ECU response.
 *
 * The class owns a fixed-size buffer to avoid dynamic allocation on the
 * sender while CAN traffic is active.
 */
class IsoTpReassembler;
```

## Error handling

- Represent protocol errors with typed status values.
- Log state changes, not every loop tick.
- Keep timeout values configurable.
- Never let UDS/capability scans block live OBD heartbeats.

## Testing expectations

Any new pure logic should have a native test. Any hardware behavior should have at least a documented manual hardware test in [18_Testing.md](18_Testing.md).

