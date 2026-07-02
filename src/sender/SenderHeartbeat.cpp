#include "SenderHeartbeat.h"

#include <Arduino.h>

#include "StatusLogic.h"
#include "config/ProjectConfig.h"
#include "config/SenderConfig.h"

namespace SenderHeartbeat {

bool tick(uint32_t nowMs,
          uint32_t& lastSentAt,
          uint32_t& heartbeatCount,
          const Input& input,
          SenderCallbacks::SendStatus sendStatus) {
    if (!input.espNowReady || sendStatus == nullptr) return false;
    if (!StatusLogic::isHeartbeatDue(nowMs, lastSentAt, SenderConfig::HeartbeatIntervalMs)) return false;

    lastSentAt = nowMs;
    ++heartbeatCount;

    char uptimeText[16];
    char heartbeatText[16];
    snprintf(uptimeText, sizeof(uptimeText), "%lu", static_cast<unsigned long>(nowMs));
    snprintf(heartbeatText, sizeof(heartbeatText), "%lu", static_cast<unsigned long>(heartbeatCount));

    sendStatus("HEARTBEAT", heartbeatText, "OK");
    sendStatus("SENDER", input.senderRunning ? "RUNNING" : "WAIT_WEB_START", input.senderRunning ? "OK" : "WARN");
    sendStatus("FW", ProjectConfig::FirmwareVersion, "OK");
    sendStatus("UPTIME", uptimeText, "OK");
    sendStatus("CAN", input.canDriverReady ? (input.canRecent ? "ACTIVE" : "IDLE") : "INIT_FAIL",
               input.canDriverReady ? (input.canRecent ? "OK" : "WARN") : "ERROR");
    sendStatus("OBD", input.obdEnabled ? (input.obdRecent ? "ACTIVE" : "NO_RESPONSE") : "DISABLED",
               input.obdEnabled ? (input.obdRecent ? "OK" : "WARN") : "WARN");
    sendStatus("UDS", input.udsEnabled ? (input.udsAvailable ? "AVAILABLE" : "UNKNOWN") : "DISABLED",
               input.udsEnabled ? (input.udsAvailable ? "OK" : "WARN") : "WARN");
    sendStatus("SIM", input.simulationEnabled ? "ACTIVE" : "INACTIVE",
               input.simulationEnabled ? "OK" : "WARN");

    return true;
}

} // namespace SenderHeartbeat
