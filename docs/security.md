# Security

## Secrets

Real credentials must be stored in `include/secrets.h`. This file is ignored by
Git. The repository only ships `include/secrets.example.h` with placeholders so
fresh checkouts still compile.

Values moved out of the public configuration:

- sender web AP SSID/password
- display web AP SSID/password
- ArduinoOTA password
- optional station WiFi credentials
- web Basic Auth credentials
- ESP-NOW LMK/AES key
- sender/display MAC addresses

## Web authentication

`SecurityConfig::EnableAuthentication` is enabled by default. Sender and display
use the shared `lib/web/AuthHelpers` module and HTTP Basic Authentication.
API clients may alternatively use the configured `Secrets::ApiToken` via one of:

- `X-API-Token: <token>`
- `Authorization: Bearer <token>`
- `?token=<token>`

Protected endpoints include:

- `/`
- `/status`
- `/log`
- `/start`
- `/restart`
- `/api/restart`
- `/api/simulation*`
- `/update`

## OTA safety

Web OTA now requires authentication and checks whether OTA sketch space is
available before starting `Update.begin()`. CI publishes stable firmware names
(`sender.bin`, `display.bin`) and SHA-256 values in `firmware_manifest.json`.
The runtime also rejects OTA uploads when placeholder web credentials,
placeholder API tokens or placeholder AP/OTA passwords from
`include/secrets.example.h` are still active. Filename validation now requires
the `.bin` suffix and a target token such as `sender.bin`,
`CANOBD2_sender_V1.2.3.bin`, `display.bin` or `CANOBD2_display_V1.2.3.bin`.
The runtime additionally inspects uploaded firmware data for embedded target and
firmware-version markers before calling the final `Update.end(true)` path and
logs the upload SHA-256 digest for comparison with CI artifacts. The current
firmware still relies on the ESP32 bootloader and partition table for binary
integrity; full cryptographic signature or manifest verification on-device
remains an open hardening step.

The active partition table is `partitions/ota_4mb.csv`, which keeps two OTA app
slots.

## Remaining risks

- Basic Auth over a SoftAP is appropriate for local maintenance but not for an
  untrusted network.
- Target validation inside `.bin` files is not yet cryptographic. A future
  manifest/signature step should bind target, version, protocol and hash to each
  firmware image and verify that manifest on-device before activation.
- The example ESP-NOW key is public and now blocks ESP-NOW startup until it is
  replaced in `include/secrets.h`.
