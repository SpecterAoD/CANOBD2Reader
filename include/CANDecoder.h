#pragma once

#include <Arduino.h>
#include <driver/twai.h>

namespace CANDecoder {

struct DecodedFrame {
    char raw[64]{};
    char hint[48]{};
};

// Erstellt eine kompakte Textdarstellung fuer die Anzeige. Ohne fahrzeugspezifische
// DBC-Datei werden Roh-ID, DLC und Bytes angezeigt; bekannte OBD-Antworten werden
// als Hinweis markiert.
DecodedFrame decode(const twai_message_t& message);

} // namespace CANDecoder
