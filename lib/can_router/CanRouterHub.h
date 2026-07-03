#pragma once

#include "CanRouter.h"

namespace CanRouting {

using FramePump = void (*)();

/// Shared CAN router used by modules that must observe the same TWAI RX stream.
///
/// Hardware access remains owned by the sender CAN driver. Consumers such as
/// ISO-TP and the passive sniffer only register listeners here, which prevents
/// multiple modules from competing for `twai_receive()`.
CanRouter& sharedRouter();

bool registerListener(CanFrameListener& listener);
bool unregisterListener(CanFrameListener& listener);
void routeFrame(const CanFrame& frame);
void clearListeners();

void setFramePump(FramePump pump);
void pumpFrames();

} // namespace CanRouting
