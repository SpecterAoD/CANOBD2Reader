#pragma once

#include <cstddef>
#include <cstdint>

#ifndef CANOBD2_FIRMWARE_VERSION
#define CANOBD2_FIRMWARE_VERSION "2.0.0"
#endif

namespace ProjectConfig {
constexpr const char* FirmwareVersion = CANOBD2_FIRMWARE_VERSION;
constexpr uint16_t ProtocolMagic = 0xCA02;
constexpr uint8_t ProtocolVersion = 1;
constexpr uint8_t EspNowChannel = 1;
constexpr uint16_t CrcInitialValue = 0xFFFF;
constexpr uint16_t CrcPolynomial = 0xA001;
constexpr std::size_t TelemetryPayloadSize = 160;
constexpr std::size_t MaxIsoTpPayload = 4095;
}
