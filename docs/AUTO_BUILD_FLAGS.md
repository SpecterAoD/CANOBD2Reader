# AUTO Build Flags

> AUTO-GENERATED FILE. Do not edit manually.
> Regenerate with the corresponding script in `scripts/`.

## PlatformIO sections

### `[platformio]`

- `default_envs`: `sender, display`

### `[env]`

- `platform`: `espressif32`
- `framework`: `arduino`
- `monitor_speed`: `115200`
- `upload_speed`: `921600`
- `lib_ldf_mode`: `chain+`
- `board_build.partitions`: `partitions/ota_4mb.csv`
- `extra_scripts`:

```text
pre:scripts/apply_firmware_version.py
pre:scripts/ensure_intelhex.py
pre:scripts/auto_upload_port.py
```

- `build_unflags`: `-std=gnu++11`
- `build_flags`:

```text
-std=gnu++17
-Wall
-Wextra
-Iinclude
-Iinclude/config
-Ilib/common
-Ilib/utils
-Ilib/telemetry
-Ilib/isotp
-Ilib/can_router
-Ilib/obd
-Ilib/uds
-Ilib/capabilities
-Ilib/simulation
-Ilib/power
-Ilib/display
-Ilib/status
-Ilib/runtime
-Ilib/web
-Ilib/update
-Ilib/network
-Ilib/logging
-Ilib/transport
-DCORE_DEBUG_LEVEL=0
-DCANOBD2_ENABLE_BLUETOOTH=0
```

- `build_src_flags`:

```text
-Wshadow
-Wdouble-promotion
-Wformat=2
-Werror=return-type
```


### `[env:sender]`

- `board`: `esp32dev`
- `custom_upload_vid`: `0x1A86`
- `custom_upload_pid`: `0x7523`
- `build_src_filter`:

```text
+<sender/>
-<display/>
```

- `build_flags`:

```text
${env.build_flags}
-DENV_SENDER
-DCANOBD2_TARGET_NAME=\"sender\"
-DCANOBD2_PROTOCOL_VERSION=2
-DCANOBD2_ENABLE_SENDER_WEBCONSOLE=1
```

- `lib_deps`: ``

### `[env:display]`

- `board`: `lilygo-t-display-s3`
- `upload_speed`: `460800`
- `custom_upload_vid`: `0x303A`
- `custom_upload_pid`: `0x1001`
- `custom_upload_serial`: `F0:F5:BD:43:29:20`
- `build_src_filter`:

```text
+<display/>
-<sender/>
```

- `build_flags`:

```text
${env.build_flags}
-DENV_DISPLAY
-DCANOBD2_TARGET_NAME=\"display\"
-DCANOBD2_PROTOCOL_VERSION=2
-DCANOBD2_ENABLE_DISPLAY_OTA=1
-DUSER_SETUP_LOADED=1
-DST7789_DRIVER=1
-DINIT_SEQUENCE_3=1
-DCGRAM_OFFSET=1
-DTFT_RGB_ORDER=TFT_RGB
-DTFT_INVERSION_ON=1
-DTFT_PARALLEL_8_BIT=1
-DTFT_WIDTH=170
-DTFT_HEIGHT=320
-DTFT_CS=6
-DTFT_DC=7
-DTFT_RST=5
-DTFT_WR=8
-DTFT_RD=9
-DTFT_D0=39
-DTFT_D1=40
-DTFT_D2=41
-DTFT_D3=42
-DTFT_D4=45
-DTFT_D5=46
-DTFT_D6=47
-DTFT_D7=48
-DTFT_BL=38
-DTFT_BACKLIGHT_ON=HIGH
```

- `lib_deps`: `bodmer/TFT_eSPI`

### `[env:native]`

- `platform`: `native`
- `framework`: ``
- `test_framework`: `unity`
- `lib_ignore`: `common`
- `build_flags`:

```text
-std=c++17
-DCANOBD2_TARGET_NAME=\"unknown\"
-DCANOBD2_PROTOCOL_VERSION=2
-Iinclude
-Iinclude/config
-Ilib/common
-Ilib/utils
-Ilib/telemetry
-Ilib/isotp
-Ilib/can_router
-Ilib/obd
-Ilib/uds
-Ilib/capabilities
-Ilib/simulation
-Ilib/power
-Ilib/display
-Ilib/status
-Ilib/runtime
-Ilib/web
-Ilib/update
-Ilib/network
-Ilib/logging
-Ilib/transport
```

- `build_src_filter`:

```text
+<../lib/utils/>
+<../lib/telemetry/>
+<../lib/isotp/>
+<../lib/can_router/>
+<../lib/obd/>
+<../lib/uds/>
+<../lib/capabilities/>
+<../lib/simulation/>
+<../lib/power/>
+<../lib/display/>
+<../lib/status/>
+<../lib/runtime/>
+<../lib/web/>
+<../lib/update/>
+<../lib/network/>
+<../lib/logging/>
+<../lib/transport/>
-<*>
```


## Common commands

```powershell
platformio run -e sender
platformio run -e display
platformio test -e native
platformio check -e sender
platformio check -e display
```
