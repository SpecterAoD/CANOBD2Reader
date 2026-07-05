# 20 - Hardware

## Contents

- [Overview](#overview)
- [Sender hardware](#sender-hardware)
- [Display hardware](#display-hardware)
- [CAN shield notes](#can-shield-notes)
- [Power](#power)
- [Hardware test checklist](#hardware-test-checklist)

## Overview

The system consists of an ESP32 sender connected to vehicle CAN/OBD-II and a LilyGO T-Display S3 dashboard.

## Sender hardware

- ESP32 DevKit V1.
- CAN transceiver/shield connected to configured TWAI RX/TX pins.
- LEDs and button according to `SenderConfig`.
- Optional voltage sensing according to configured ADC pin.

## Display hardware

- LilyGO T-Display S3.
- Built-in TFT and buttons.
- Backlight controlled according to `DisplayConfig`.

## CAN shield notes

The MR DIY ESP32 DevKit CAN shield schematic suggests the software must treat CAN TX/RX pins, power rail behavior and transceiver enable/standby pins as hardware-specific configuration. Verify actual pins against `SenderConfig` before relying on vehicle tests.

## Power

Vehicle power can remain active for minutes after ignition off, especially on VW MQB/MQB Evo vehicles. Power management must rely on activity scoring, not only RPM.

## Hardware test checklist

- Sender boots from vehicle power.
- CAN-H/CAN-L polarity correct.
- OBD requests visible in diagnostic log.
- ECU responses received.
- ESP-NOW heartbeat received by display.
- Web AP appears reliably.
- OTA works for both targets.

