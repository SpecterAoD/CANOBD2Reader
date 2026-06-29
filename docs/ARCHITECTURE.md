# Architecture

The repository is one PlatformIO project with two firmware environments:

- `sender`: ESP32 DevKit V1, TWAI/CAN, OBD-II polling, ESP-NOW telemetry, OTA and web console
- `display`: LilyGO T-Display S3, ESP-NOW receive queue, dashboard UI and OTA upload page

```text
sender
  CANHandler -> OBDHandler -> IsoTpHandler -> PidDecoder
       |             |              |
       +-------------+--------------+
                     |
              TelemetryCodec
                     |
                  ESP-NOW
                     |
display onReceive -> FreeRTOS queue -> TelemetryCodec -> DisplayData -> DisplayUi
```

## Shared modules

- `lib/utils`: CRC16 and small common utilities
- `lib/obd`: PID decoding, independent from Arduino where possible
- `lib/isotp`: ISO 15765-2 reassembly and Arduino/TWAI transport wrapper
- `lib/telemetry`: one packet format, CRC validation and shared sequence source
- `lib/common`: compatibility layer and display/sender shared payload helpers

## Configuration

Configuration is split by scope:

- `include/ProjectConfig.h`: firmware/protocol identity and shared limits
- `include/BuildConfig.h`: compile-time feature switches
- `include/SenderConfig.h`: sender-side CAN/OBD timing defaults
- `include/DisplayConfig.h`: display refresh and queue defaults
- `include/Config.h`: active application configuration used by the current firmware

`Config.h` still contains some legacy namespaces for compatibility. New code
should prefer the narrower config headers when no existing module dependency is
involved.

## OTA

Both firmware environments use `board_build.partitions = min_spiffs.csv`, which
keeps two OTA application slots on the supported ESP32 boards. Web OTA upload is
available from the sender web console and the display OTA page.

## Legacy code

Old Arduino-IDE sketches and removed modules are stored under `docs/archive/`.
They are not part of `build_src_filter` and cannot define active `setup()` or
`loop()` functions.
