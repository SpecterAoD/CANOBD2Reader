#pragma once

#include <cstddef>
#include <cstdint>

#include "common_config.h"

namespace ProjectConfig {
constexpr const char* FirmwareVersion = CANOBD2_FIRMWARE_VERSION;
constexpr uint16_t ProtocolMagic = 0xCA02;
constexpr uint16_t TelemetryMagic = ProtocolMagic;
constexpr uint8_t ProtocolVersion = CANOBD2_PROTOCOL_VERSION;
constexpr const char* TargetName = CANOBD2_TARGET_NAME;
constexpr int Baudrate = 115200;
constexpr uint8_t EspNowChannel = 1;
constexpr uint16_t CrcInitialValue = 0xFFFF;
constexpr uint16_t CrcPolynomial = 0xA001;
constexpr std::size_t TelemetryPayloadSize = 160;
constexpr std::size_t MaxIsoTpPayload = 4095;
}
