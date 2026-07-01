# Simulation

Simulation is runtime-only and disabled after every boot:

```cpp
SimulationConfig::EnableSimulationByDefault == false
```

The web UI can enable it temporarily and select a scenario. No simulation state
is stored in NVS, EEPROM or SPIFFS.

Implemented scenarios:

- `NormalSingleFrame`
- `NormalMultiFrameVin`
- `NormalMultiFrameDtc`
- `FlowControlRequired`
- `TimeoutAfterFirstFrame`
- `SequenceError`
- `BufferOverflow`
- `MultipleEcusResponse`
- `NegativeResponse`
- `DisplayNormalValues`
- `DisplayWarningValues`
- `DisplayCriticalValues`
- `DisplayTimeoutValues`
- `DisplayMixedValues`

Display scenarios include speed, RPM, coolant, oil, voltage, MAP, BARO and
boost values so dashboard colors and timeout behavior can be tested without a
vehicle.
