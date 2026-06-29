#include "Crc16.h"

namespace Crc16 {

uint16_t modbus(const uint8_t* data, std::size_t length, uint16_t initial) {
    uint16_t crc = initial;
    for (std::size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1U) ^ 0xA001U)
                             : static_cast<uint16_t>(crc >> 1U);
        }
    }
    return crc;
}

}
