# 15 - Webserver

## Contents

- [Overview](#overview)
- [Current state](#current-state)
- [Pages](#pages)
- [Shared handlers](#shared-handlers)
- [REST APIs](#rest-apis)
- [Target state](#target-state)

## Overview

Sender and display expose web pages for status, diagnostics, simulation, restart and OTA.

## Current state

Shared helpers live in `lib/web/`. Sender-specific console code lives in `src/sender/WebConsoleHandler.cpp`; display OTA web handling lives in `src/display/DisplayOta.cpp`.

## Pages

Sender web pages:

- status,
- diagnostics,
- log,
- OTA,
- capabilities.

Display web pages:

- status,
- OTA,
- simulation/status where enabled.

## Shared handlers

Shared code should cover:

- authentication,
- restart,
- simulation state,
- OTA validation,
- JSON escaping,
- common status fields.

## REST APIs

The API reference is maintained in [21_API.md](21_API.md).

## Target state

Reduce duplicate HTML/JS between sender and display. Buttons must show visible status and errors after every API call.

