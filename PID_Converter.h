#ifndef PID_CONVERTER_H
#define PID_CONVERTER_H

#include <Arduino.h>

// Struktur für umgerechnete Werte
struct PIDResult {
    float value;
    const char* unit;
};

// Hauptfunktion zur Umrechnung von Rohdaten
PIDResult convertPID(byte pid, const byte* data, byte length);

// Gibt lesbaren Namen zu einem PID zurück
const char* getPIDName(byte pid);

#endif
