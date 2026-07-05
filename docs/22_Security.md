# 22 - Security

## Contents

- [Overview](#overview)
- [Secrets](#secrets)
- [Web authentication](#web-authentication)
- [OTA security](#ota-security)
- [Diagnostics safety](#diagnostics-safety)
- [Open risks](#open-risks)

## Overview

The project runs on a local embedded network, but OTA, restart and diagnostic actions still need protection.

## Secrets

- Never commit `include/secrets.h`.
- Use `include/secrets.example.h` for CI placeholder builds.
- `SecurityConfig::BlockNetworkFeaturesOnPlaceholderSecrets` controls whether placeholder secrets block network features.

## Web authentication

Authentication should protect:

- OTA,
- restart,
- simulation toggles,
- configuration changes,
- capability scanner actions.

## OTA security

Current OTA validation checks:

- target metadata,
- version metadata,
- protocol marker,
- filename hint,
- upload/write success.

Future target:

- signed metadata,
- stronger manifest validation,
- optional release-channel enforcement.

## Diagnostics safety

Capability scans must be manually started. UDS must remain read-only and conservative by default.

## Open risks

- Basic Authentication is not strong against local attackers.
- Firmware metadata is currently integrity marker, not cryptographic signature.
- ESP-NOW encryption depends on correct local secrets.

