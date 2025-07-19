#ifndef PID_CONVERTER_H
#define PID_CONVERTER_H

#include <Arduino.h>

// Ergebnisstruktur für berechnete Werte
struct PIDResult {
    float value;
    const char* unit;
};

// Hauptschnittstelle: Liefert berechneten Wert und Einheit
PIDResult convertPID(byte pid, const byte* data, byte length);

#endif