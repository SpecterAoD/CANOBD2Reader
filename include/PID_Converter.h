#ifndef PID_CONVERTER_H
#define PID_CONVERTER_H

#include <cstdint>
#include "PidDecoder.h"

// Struktur für umgerechnete Werte
using PIDResult = Obd::PidValue;

// Hauptfunktion zur Umrechnung von Rohdaten
PIDResult convertPID(uint8_t pid, const uint8_t* data, uint8_t length);

// Gibt lesbaren Namen zu einem PID zurück
const char* getPIDName(uint8_t pid);

#endif
