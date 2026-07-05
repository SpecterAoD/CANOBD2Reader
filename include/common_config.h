#pragma once

// Zentrale PlatformIO-Konfiguration fuer beide Firmware-Ziele.
// Environment-spezifische Schalter kommen aus platformio.ini per -DENV_SENDER
// bzw. -DENV_DISPLAY. Gemeinsame Projektkonstanten bleiben hier an einer Stelle.

#ifndef CANOBD2_PROTOCOL_VERSION
  #define CANOBD2_PROTOCOL_VERSION 1
#endif

#ifndef CANOBD2_FIRMWARE_VERSION
  #define CANOBD2_FIRMWARE_VERSION "V0.0.0-dev"
#endif

#ifndef CANOBD2_TARGET_NAME
  #if defined(ENV_SENDER)
    #define CANOBD2_TARGET_NAME "sender"
  #elif defined(ENV_DISPLAY)
    #define CANOBD2_TARGET_NAME "display"
  #else
    #define CANOBD2_TARGET_NAME "unknown"
  #endif
#endif

#define CANOBD2_STRINGIFY_VALUE(value) #value
#define CANOBD2_STRINGIFY(value) CANOBD2_STRINGIFY_VALUE(value)

#ifndef CANOBD2_OTA_COMPAT_VERSION_MARKERS
  #define CANOBD2_OTA_COMPAT_VERSION_MARKERS "V1.0.13;V2.0.0.b1;V2.0.0.b2;V2.0.0.b3;V0.0.0-dev"
#endif

// OTA bleibt pro Environment aktiviert; die konkrete Nutzung/Upload-Methode wird
// nicht im Code erzwungen, sondern ueber PlatformIO upload_protocol gesteuert.
#define CANOBD2_OTA_COMPATIBLE 1

#ifndef CANOBD2_ENABLE_BLUETOOTH
#define CANOBD2_ENABLE_BLUETOOTH 0
#endif

#ifndef CANOBD2_ENABLE_SENDER_WEBCONSOLE
#define CANOBD2_ENABLE_SENDER_WEBCONSOLE 1
#endif

#ifndef CANOBD2_ENABLE_DISPLAY_OTA
#define CANOBD2_ENABLE_DISPLAY_OTA 1
#endif
