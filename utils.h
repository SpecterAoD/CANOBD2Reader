#pragma once
#include <Arduino.h>
#include "Config.h"  // Enthält deine Pins, Structs, Defines

namespace Utils {

    // ============ CRC ============

    /// @brief Berechnet CRC16 (Modbus) für ein Byte-Array
    /// @param data Pointer auf die Daten
    /// @param length Länge der Daten
    /// @return CRC16-Wert
    inline uint16_t crc16(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; ++i) {
            crc ^= data[i];
            for (int j = 0; j < 8; ++j) {
                if (crc & 1)
                    crc = (crc >> 1) ^ 0xA001;
                else
                    crc >>= 1;
            }
        }
        return crc;
    }
    
    
}