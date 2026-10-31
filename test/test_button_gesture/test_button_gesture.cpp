// Display button: debouncing and press classification.
#include <unity.h>

#include "ButtonGesture.h"

using DisplayLogic::ButtonEvent;
using DisplayLogic::ButtonGesture;

void setUp() {}
void tearDown() {}

namespace {
// Holds the raw level from `from` to `to` (exclusive) in 5 ms steps and
// returns the last non-None event.
ButtonEvent hold(ButtonGesture& b, bool level, uint32_t from, uint32_t to) {
    ButtonEvent last = ButtonEvent::None;
    for (uint32_t t = from; t < to; t += 5) {
        const ButtonEvent e = b.update(level, t);
        if (e != ButtonEvent::None) last = e;
    }
    return last;
}
} // namespace

void test_quick_tap_is_short_press() {
    ButtonGesture b(30, 1200, 3000);
    TEST_ASSERT_EQUAL(static_cast<int>(ButtonEvent::None), static_cast<int>(hold(b, true, 0, 120)));
    TEST_ASSERT_EQUAL(static_cast<int>(ButtonEvent::Short), static_cast<int>(hold(b, false, 120, 200)));
}

void test_bounce_is_ignored() {
    ButtonGesture b(30, 1200, 3000);
    // 10 ms spikes never become a press.
    for (uint32_t t = 0; t < 200; t += 20) {
        TEST_ASSERT_EQUAL(static_cast<int>(ButtonEvent::None), static_cast<int>(b.update(true, t)));
        TEST_ASSERT_EQUAL(static_cast<int>(ButtonEvent::None), static_cast<int>(b.update(false, t + 10)));
    }
    TEST_ASSERT_FALSE(b.pressed());
}

void test_long_press_on_release() {
    ButtonGesture b(30, 1200, 3000);
    hold(b, true, 0, 1500);
    TEST_ASSERT_EQUAL(static_cast<int>(ButtonEvent::Long), static_cast<int>(hold(b, false, 1500, 1600)));
}

void test_very_long_press_fires_while_holding_once() {
    ButtonGesture b(30, 1200, 3000);
    TEST_ASSERT_EQUAL(static_cast<int>(ButtonEvent::VeryLong), static_cast<int>(hold(b, true, 0, 3500)));
    TEST_ASSERT_EQUAL(static_cast<int>(ButtonEvent::None), static_cast<int>(hold(b, false, 3500, 3600)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_quick_tap_is_short_press);
    RUN_TEST(test_bounce_is_ignored);
    RUN_TEST(test_long_press_on_release);
    RUN_TEST(test_very_long_press_fires_while_holding_once);
    return UNITY_END();
}
