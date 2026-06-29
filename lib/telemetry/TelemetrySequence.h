#pragma once

#include <cstdint>

namespace Telemetry {

/**
 * Provides one process-wide telemetry sequence counter.
 *
 * Sender modules use this instead of local static counters so that packet-loss
 * estimation on the display sees a monotonic stream across OBD, CAN, status and
 * diagnostic packets.
 */
uint32_t nextSequence();

}
