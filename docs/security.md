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
available before starting `Update.begin()`. The current firmware still relies on
the ESP32 bootloader and partition table for binary integrity; target-specific
firmware metadata validation remains an open hardening step.

The active partition table is `partitions/ota_4mb.csv`, which keeps two OTA app
slots.

## Remaining risks

- Basic Auth over a SoftAP is appropriate for local maintenance but not for an
  untrusted network.
- Target validation inside `.bin` files is not yet cryptographic. A future
  manifest/signature step should bind target, version, protocol and hash to each
  firmware image.
- The example ESP-NOW key is public and must be replaced in `include/secrets.h`.
