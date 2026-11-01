#pragma once

// Crash diagnostics and loop watchdog (sender and display).
//
//  * Reset reason and a stored core dump are reported once at boot
//    ("[BOOT] reset=... coredump: task=... pc=0x...").
//  * Core dumps (ELF, partition "coredump") can be downloaded via
//    GET /api/coredump and analysed with
//      esp-coredump info_corefile -t raw -c coredump.bin firmware.elf
//  * The loop task watchdog resets the device if loop() hangs for
//    WatchdogTimeoutMs; together with BootValidation a hanging new firmware
//    is rolled back. Long blocking operations call SystemHealth::feed().

#include <cstddef>
#include <cstdint>

namespace SystemHealth {

constexpr uint32_t WatchdogTimeoutMs = 30000;

// Reset reason as short text, e.g. "POWERON", "PANIC", "TASK_WDT".
const char* resetReasonText();
bool crashReset();

struct CoreDumpInfo {
    bool present = false;
    std::size_t address = 0;
    std::size_t size = 0;
    char task[16] = {};
    uint32_t pc = 0;
};

CoreDumpInfo coreDump();
bool eraseCoreDump();
// Reads `length` bytes of the stored core dump starting at `offset`.
bool readCoreDump(std::size_t offset, uint8_t* buffer, std::size_t length);

// Writes the boot summary to the diagnostic log (call after DiagnosticLog::begin).
void logBootSummary(const char* device);

// Station MAC from eFuse (valid before WiFi is started), "aa:bb:cc:dd:ee:ff".
const char* macAddress();

void startWatchdog();
void feed();

} // namespace SystemHealth
