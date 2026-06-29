#include "TelemetrySequence.h"

namespace Telemetry {

namespace {
uint32_t sequence = 0;
}

uint32_t nextSequence() {
    return ++sequence;
}

}
