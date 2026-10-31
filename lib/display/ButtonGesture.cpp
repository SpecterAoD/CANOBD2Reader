#include "ButtonGesture.h"

namespace DisplayLogic {

ButtonEvent ButtonGesture::update(bool rawPressed, uint32_t nowMs) {
    if (rawPressed != raw_) {
        raw_ = rawPressed;
        rawChangedAt_ = nowMs;
    }

    if (raw_ != stable_ && nowMs - rawChangedAt_ >= debounceMs_) {
        stable_ = raw_;
        if (stable_) {
            pressedAt_ = rawChangedAt_;
            veryLongReported_ = false;
            return ButtonEvent::None;
        }
        // Released: classify by how long it was held.
        if (veryLongReported_) return ButtonEvent::None;
        return rawChangedAt_ - pressedAt_ >= longMs_ ? ButtonEvent::Long : ButtonEvent::Short;
    }

    if (stable_ && !veryLongReported_ && nowMs - pressedAt_ >= veryLongMs_) {
        veryLongReported_ = true;
        return ButtonEvent::VeryLong;
    }
    return ButtonEvent::None;
}

} // namespace DisplayLogic
