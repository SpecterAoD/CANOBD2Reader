#pragma once

#include <cstddef>
#include <cstdint>

namespace CanRouting {

struct CanFrame {
    uint32_t id = 0;
    uint8_t length = 0;
    uint8_t data[8]{};
    uint32_t timestampMs = 0;
};

class CanFrameListener {
public:
    virtual ~CanFrameListener() = default;
    virtual void onCanFrame(const CanFrame& frame) = 0;
};

class CanRouter {
public:
    static constexpr std::size_t MaxListeners = 6;

    bool registerListener(CanFrameListener& listener);
    bool unregisterListener(CanFrameListener& listener);
    std::size_t listenerCount() const;
    void route(const CanFrame& frame);
    void clear();

private:
    CanFrameListener* listeners_[MaxListeners]{};
    std::size_t listenerCount_ = 0;
};

}
