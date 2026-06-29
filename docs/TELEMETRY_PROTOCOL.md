# Telemetry Protocol

Sender and display exchange one canonical ESP-NOW packet type:
`Telemetry::TelemetryPacket` from `lib/telemetry/TelemetryPacket.h`.

## Binary packet

```cpp
struct TelemetryPacket {
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint32_t sequence;
    uint32_t timestamp;
    uint16_t payloadLength;
    uint8_t payload[ProjectConfig::TelemetryPayloadSize];
    uint16_t crc;
};
```

Validation is centralized in `TelemetryCodec`:

- fixed packet size
- magic number `ProjectConfig::ProtocolMagic`
- protocol version `ProjectConfig::ProtocolVersion`
- payload length
- CRC16/Modbus over the header and payload fields before `crc`

Invalid packets are dropped by the display before UI state is updated.

## Text payload

The payload remains a compact CSV line so the web console, serial output and
display debug page stay readable:

```text
TYPE,KEY,NAME,VALUE,UNIT,STATUS,SEQ
```

The packet header sequence is authoritative for packet-loss detection. The CSV
sequence is kept for diagnostics only.

Examples:

```text
OBD,0C,RPM,1850.00,rpm,OK,42
OBD,05,CoolantTemp,89.00,degC,OK,43
BATTERY,VOLTAGE,BatteryVoltage,12.64,V,OK,44
STATUS,CAN,CAN,ACTIVE,,OK,45
CAN,RAW,LastCAN,0x7E8 DLC8 04 41 0C 1A F8 55 55 55,,OK,46
DTC,ACTIVE,DTC,P0133 P0420,,WARN,47
```

## Packet types

- `Text`
- `Status`
- `Obd`
- `Can`
- `Diagnostic`

## Display fields

- Main: `Speed`, `RPM`, `CoolantTemp`, `BatteryVoltage`
- Engine: `OilTemp`, `EngineLoad`, `Throttle`, `IntakeTemp`
- Consumption: `InstantConsumption`, `AverageConsumption`, `FuelRate`
- Additional values: `MAF`, `FuelLevel`, `RunTime`, `AmbientTemp`
- CAN page: `LastCAN`, `CANHint`, `CANCount`
- Diagnostics: CAN/OBD/ESP-NOW status, sequence, packet loss, firmware version
- Error codes: `DTC`

## Backward compatibility

The display still accepts a few old CSV shapes during migration, but new sender
code must send `TelemetryPacket` only. New logic must not introduce additional
ESP-NOW frame formats.
