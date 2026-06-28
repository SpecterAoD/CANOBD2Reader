# Telemetry Protocol

Sender and display use the same ESP-NOW text frame:

```cpp
typedef struct {
  char payload[128];
  uint16_t crc;
} esp_now_text_frame_t;
```

The CRC is calculated over the struct excluding the final `crc` field.

Payload format:

```text
TYPE,KEY,NAME,VALUE,UNIT,STATUS,SEQ
```

Fields:

- `TYPE`: `OBD`, `BATTERY`, `FUEL`, `DTC`, or `STATUS`
- `KEY`: PID hex value or logical key
- `NAME`: stable display name
- `VALUE`: numeric value or `N/A`
- `UNIT`: physical unit, empty for pure status values
- `STATUS`: `OK`, `WARN`, `ALERT`, `TIMEOUT`, `SEND_FAIL`, or `ERROR`
- `SEQ`: sender-side packet counter for packet-loss estimation

Current display pages consume these names:

- Main: `Speed`, `RPM`, `CoolantTemp`, `BatteryVoltage`
- Engine: `OilTemp`, `CoolantTemp`, `EngineLoad`, `IntakeTemp`
- Consumption: `AverageConsumption`, `FuelRate`, `Speed`, `Throttle`
- Additional values: `MAF`, `FuelLevel`, `RunTime`, `AmbientTemp`
- CAN page: `LastCAN`, `CANHint`, `CANCount`
- Diagnostics: `CAN`, packet counters, CRC errors, last payload
- Error codes: `DTC`

The sender periodically queries supported PID ranges (`0x00`, `0x20`, `0x40`) and skips unsupported live-data PIDs after support detection. DTCs are queried with OBD mode `0x03` and sent as a space-separated code list.

Simulation:

- Sender flag: `ENABLE_TELEMETRY_SIMULATION`
- Display-only flag: `ENABLE_DISPLAY_INTERNAL_SIMULATION`
- Shared simulated values: `include/SimulationData.h`

Backward compatibility:

- Old battery packets like `BATTERY,VOLTAGE,12.4,V` are still accepted.
- Old three-field OBD packets like `0C,RPM,1800 rpm` are still accepted, but provide less status information.
