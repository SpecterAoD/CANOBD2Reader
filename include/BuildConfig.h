#pragma once

#include <cstdint>

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

constexpr bool BluetoothEnabled = false;
constexpr bool OtaCompatible = true;
}
