#pragma once

#include <cstddef>
#include <cstdint>

namespace Crc16 {
uint16_t modbus(const uint8_t* data, std::size_t length, uint16_t initial = 0xFFFF);
}
