#pragma once

// Display web UI (status, page switching) on top of the shared management
// server (signed updates, WiFi, logs, restart).
namespace DisplayWeb {
void begin();
void handle();
}
