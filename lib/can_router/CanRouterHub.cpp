#include "CanRouterHub.h"

namespace {
CanRouting::CanRouter router;
CanRouting::FramePump framePump = nullptr;
}

namespace CanRouting {

CanRouter& sharedRouter() {
    return router;
}

bool registerListener(CanFrameListener& listener) {
    return router.registerListener(listener);
}

bool unregisterListener(CanFrameListener& listener) {
    return router.unregisterListener(listener);
}

void routeFrame(const CanFrame& frame) {
    router.route(frame);
}

void clearListeners() {
    router.clear();
}

void setFramePump(FramePump pump) {
    framePump = pump;
}

void pumpFrames() {
    if (framePump != nullptr) framePump();
}

} // namespace CanRouting
