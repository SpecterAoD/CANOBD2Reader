#include "CanRouter.h"

namespace CanRouting {

bool CanRouter::registerListener(CanFrameListener& listener) {
    for (std::size_t index = 0; index < listenerCount_; ++index) {
        if (listeners_[index] == &listener) return true;
    }

    if (listenerCount_ >= MaxListeners) return false;
    listeners_[listenerCount_++] = &listener;
    return true;
}

bool CanRouter::unregisterListener(CanFrameListener& listener) {
    for (std::size_t index = 0; index < listenerCount_; ++index) {
        if (listeners_[index] != &listener) continue;

        for (std::size_t move = index + 1; move < listenerCount_; ++move) {
            listeners_[move - 1] = listeners_[move];
        }
        listeners_[--listenerCount_] = nullptr;
        return true;
    }
    return false;
}

std::size_t CanRouter::listenerCount() const {
    return listenerCount_;
}

void CanRouter::route(const CanFrame& frame) {
    for (std::size_t index = 0; index < listenerCount_; ++index) {
        if (listeners_[index] != nullptr) {
            listeners_[index]->onCanFrame(frame);
        }
    }
}

void CanRouter::clear() {
    for (std::size_t index = 0; index < listenerCount_; ++index) {
        listeners_[index] = nullptr;
    }
    listenerCount_ = 0;
}

}
