#pragma once
#include <Arduino.h>
#include <esp_now.h>
#include "Config.h"

namespace Utils {

    // ============ CRC ============
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

    // ============ ESP-NOW SEND ============

    /// @brief Sendet ein Text-Frame über ESP-NOW
    inline void sendTextFrame(const char* payload) {
        using namespace Config;

        strncpy(textFrame.payload, payload, sizeof(textFrame.payload));
        textFrame.payload[sizeof(textFrame.payload)-1] = '\0';
        textFrame.crc = crc16(reinterpret_cast<uint8_t*>(&textFrame), sizeof(textFrame) - 2);

        esp_now_send(EspNowPeerMac, reinterpret_cast<uint8_t*>(&textFrame), sizeof(textFrame));
    }

    /// @brief Sendet CAN-Datenframe über ESP-NOW
    inline void sendCanFrame(uint32_t canId, const uint8_t* data, uint8_t len) {
        using namespace Config;

        canFrame.timestamp = millis();
        canFrame.meshId = 0; // Optional, falls Mesh ID später hinzugefügt wird
        canFrame.canId = canId;
        canFrame.len = len > 8 ? 8 : len;
        memcpy(canFrame.data, data, canFrame.len);

        canFrame.crc = crc16(reinterpret_cast<uint8_t*>(&canFrame), sizeof(canFrame) - 2);
        esp_now_send(EspNowPeerMac, reinterpret_cast<uint8_t*>(&canFrame), sizeof(canFrame));
    }

    /// @brief Sendet ein OBD2-Rohdatenframe als Hex-String
    inline void sendRawOBDData(byte pid, const uint8_t* data, byte len) {
        char buffer[64];
        size_t offset = 0;

        offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%02X,", pid);
        for (byte i = 0; i < len; ++i) {
            offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%02X ", data[i]);
        }

        sendTextFrame(buffer);
    }

    /// @brief Sendet ein OBD2-Ergebnis (berechneter Wert + Einheit)
    inline void sendOBDValue(byte pid, const char* name, float value, const char* unit) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%02X,%s,%.2f,%s", pid, name, value, unit);
        sendTextFrame(buffer);
    }

    /// @brief Sendet ein OBD2-Fehlerframe
    inline void sendOBDError(byte pid, const char* name) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "%02X,%s,ERROR,N/A", pid, name);
        sendTextFrame(buffer);
    }

}