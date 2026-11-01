#include "SystemHealth.h"

#if defined(ARDUINO)
  #include <Arduino.h>
  #include <esp_core_dump.h>
  #include <esp_flash.h>
  #include <esp_mac.h>
  #include <esp_system.h>
  #include <esp_task_wdt.h>
  #include "DiagnosticLog.h"
#endif

#include <cstdio>
#include <cstring>

namespace SystemHealth {

const char* resetReasonText() {
#if defined(ARDUINO)
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON: return "POWERON";
        case ESP_RST_EXT: return "EXTERNAL";
        case ESP_RST_SW: return "SOFTWARE";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT: return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_SDIO: return "SDIO";
        case ESP_RST_USB: return "USB";
        default: return "UNKNOWN";
    }
#else
    return "NATIVE";
#endif
}

bool crashReset() {
#if defined(ARDUINO)
    const esp_reset_reason_t reason = esp_reset_reason();
    return reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT ||
           reason == ESP_RST_WDT || reason == ESP_RST_BROWNOUT;
#else
    return false;
#endif
}

CoreDumpInfo coreDump() {
    CoreDumpInfo info;
#if defined(ARDUINO)
    if (esp_core_dump_image_check() != ESP_OK) return info;
    if (esp_core_dump_image_get(&info.address, &info.size) != ESP_OK) return info;
    info.present = true;
    esp_core_dump_summary_t* summary =
        static_cast<esp_core_dump_summary_t*>(malloc(sizeof(esp_core_dump_summary_t)));
    if (summary != nullptr) {
        if (esp_core_dump_get_summary(summary) == ESP_OK) {
            std::snprintf(info.task, sizeof(info.task), "%s", summary->exc_task);
            info.pc = summary->exc_pc;
        }
        free(summary);
    }
#endif
    return info;
}

bool eraseCoreDump() {
#if defined(ARDUINO)
    return esp_core_dump_image_erase() == ESP_OK;
#else
    return false;
#endif
}

bool readCoreDump(std::size_t offset, uint8_t* buffer, std::size_t length) {
#if defined(ARDUINO)
    const CoreDumpInfo info = coreDump();
    if (!info.present || offset + length > info.size) return false;
    return esp_flash_read(nullptr, buffer, info.address + offset, length) == ESP_OK;
#else
    (void)offset;
    (void)buffer;
    (void)length;
    return false;
#endif
}

void logBootSummary(const char* device) {
#if defined(ARDUINO)
    const CoreDumpInfo dump = coreDump();
    if (dump.present) {
        DiagnosticLog::appendf("[BOOT] %s reset=%s, Absturzbericht vorhanden: task=%s pc=0x%08lx (%u Bytes, /api/coredump)",
                               device, resetReasonText(), dump.task, static_cast<unsigned long>(dump.pc),
                               static_cast<unsigned>(dump.size));
    } else {
        DiagnosticLog::appendf("[BOOT] %s reset=%s", device, resetReasonText());
    }
#else
    (void)device;
#endif
}

const char* macAddress() {
    static char text[18] = "00:00:00:00:00:00";
#if defined(ARDUINO)
    uint8_t mac[6] = {};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        std::snprintf(text, sizeof(text), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4],
                      mac[5]);
    }
#endif
    return text;
}

void startWatchdog() {
#if defined(ARDUINO)
    // The task watchdog is initialised by the core with 5 s; give loop() more
    // room for TLS handshakes and UDS "response pending" sequences.
    esp_task_wdt_config_t config = {};
    config.timeout_ms = WatchdogTimeoutMs;
    config.idle_core_mask = 0;  // only watch the loop task, not the idle tasks
    config.trigger_panic = true; // panic -> core dump -> reboot
    esp_task_wdt_reconfigure(&config);
    enableLoopWDT();
#endif
}

void feed() {
#if defined(ARDUINO)
    feedLoopWDT();
#endif
}

} // namespace SystemHealth
