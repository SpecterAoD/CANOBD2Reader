#pragma once

#include <cstdint>
#include "common_config.h"

namespace BuildConfig {
enum class LogLevel : uint8_t {
    Error = 0,
    Warn = 1,
    Info = 2,
    Debug = 3
};

#ifndef CANOBD2_LOG_LEVEL
constexpr LogLevel CompileTimeLogLevel = LogLevel::Info;
#else
constexpr LogLevel CompileTimeLogLevel = static_cast<LogLevel>(CANOBD2_LOG_LEVEL);
#endif

constexpr bool BluetoothEnabled = CANOBD2_ENABLE_BLUETOOTH;
constexpr bool SenderWebConsoleEnabled = CANOBD2_ENABLE_SENDER_WEBCONSOLE;
constexpr bool DisplayOtaEnabled = CANOBD2_ENABLE_DISPLAY_OTA;
constexpr bool OtaCompatible = CANOBD2_OTA_COMPATIBLE;
}
