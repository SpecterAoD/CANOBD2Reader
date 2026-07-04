#include <unity.h>

#include "SenderLedController.h"

namespace {

Status::SenderLedInput baseInput(uint32_t nowMs = 1000) {
    Status::SenderLedInput input{};
    input.nowMs = nowMs;
    input.senderRunning = true;
    input.canDriverReady = true;
    input.espNowReady = true;
    return input;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_booting_blinks_green_and_has_no_error_led() {
    Status::SenderLedController controller;
    controller.reset(0);

    Status::SenderLedInput input{};
    input.nowMs = 0;
    const Status::SenderLedOutput output = controller.update(input);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Status::SenderLedState::Booting),
                            static_cast<uint8_t>(output.state));
    TEST_ASSERT_TRUE(output.greenOn);
    TEST_ASSERT_FALSE(output.errorOn);
}

void test_ready_keeps_green_led_on() {
    Status::SenderLedController controller;
    controller.reset(0);

    const Status::SenderLedOutput output = controller.update(baseInput());

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Status::SenderLedState::Ready),
                            static_cast<uint8_t>(output.state));
    TEST_ASSERT_TRUE(output.greenOn);
    TEST_ASSERT_FALSE(output.errorOn);
}

void test_vehicle_off_after_previous_can_activity_uses_vehicle_off_state() {
    Status::SenderLedController controller;
    controller.reset(0);

    Status::SenderLedInput input = baseInput(1000);
    input.canActive = true;
    controller.update(input);

    input.nowMs = 2000;
    input.canActive = false;
    const Status::SenderLedOutput output = controller.update(input);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Status::SenderLedState::VehicleOff),
                            static_cast<uint8_t>(output.state));
    TEST_ASSERT_TRUE(output.vehicleOff);
    TEST_ASSERT_FALSE(output.errorOn);
}

void test_can_activity_after_vehicle_off_restores_green_led() {
    Status::SenderLedController controller;
    controller.reset(0);

    Status::SenderLedInput input = baseInput(1000);
    input.canActive = true;
    controller.update(input);
    input.nowMs = 2000;
    input.canActive = false;
    controller.update(input);

    input.nowMs = 3000;
    input.canActive = true;
    const Status::SenderLedOutput output = controller.update(input);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Status::SenderLedState::CanActive),
                            static_cast<uint8_t>(output.state));
    TEST_ASSERT_TRUE(output.greenOn);
    TEST_ASSERT_FALSE(output.errorOn);
    TEST_ASSERT_FALSE(output.vehicleOff);
}

void test_esp_now_error_uses_error_led() {
    Status::SenderLedController controller;
    controller.reset(0);

    Status::SenderLedInput input = baseInput(0);
    input.espNowReady = false;
    const Status::SenderLedOutput output = controller.update(input);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Status::SenderLedState::EspNowError),
                            static_cast<uint8_t>(output.state));
    TEST_ASSERT_FALSE(output.greenOn);
    TEST_ASSERT_TRUE(output.errorOn);
}

void test_can_error_turns_error_led_on() {
    Status::SenderLedController controller;
    controller.reset(0);

    Status::SenderLedInput input = baseInput();
    input.canDriverReady = false;
    const Status::SenderLedOutput output = controller.update(input);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Status::SenderLedState::CanError),
                            static_cast<uint8_t>(output.state));
    TEST_ASSERT_FALSE(output.greenOn);
    TEST_ASSERT_TRUE(output.errorOn);
}

void test_led_test_is_temporary_and_restores_previous_status() {
    Status::SenderLedController controller;
    controller.reset(0);

    Status::SenderLedInput input = baseInput(1000);
    Status::SenderLedOutput output = controller.update(input);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Status::SenderLedState::Ready),
                            static_cast<uint8_t>(output.state));

    input.nowMs = 1100;
    input.ledTestActive = true;
    output = controller.update(input);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Status::SenderLedState::LedTest),
                            static_cast<uint8_t>(output.state));
    TEST_ASSERT_TRUE(output.greenOn);
    TEST_ASSERT_TRUE(output.errorOn);

    input.nowMs = 1200;
    input.ledTestActive = false;
    output = controller.update(input);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Status::SenderLedState::Ready),
                            static_cast<uint8_t>(output.state));
    TEST_ASSERT_TRUE(output.greenOn);
    TEST_ASSERT_FALSE(output.errorOn);
}

void test_error_pulse_does_not_turn_green_sender_alive_led_off() {
    Status::SenderLedController controller;
    controller.reset(0);

    Status::SenderLedInput input = baseInput(1000);
    controller.requestErrorPulse(1000);
    const Status::SenderLedOutput output = controller.update(input);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Status::SenderLedState::Ready),
                            static_cast<uint8_t>(output.state));
    TEST_ASSERT_TRUE(output.greenOn);
    TEST_ASSERT_TRUE(output.errorOn);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_booting_blinks_green_and_has_no_error_led);
    RUN_TEST(test_ready_keeps_green_led_on);
    RUN_TEST(test_vehicle_off_after_previous_can_activity_uses_vehicle_off_state);
    RUN_TEST(test_can_activity_after_vehicle_off_restores_green_led);
    RUN_TEST(test_esp_now_error_uses_error_led);
    RUN_TEST(test_can_error_turns_error_led_on);
    RUN_TEST(test_led_test_is_temporary_and_restores_previous_status);
    RUN_TEST(test_error_pulse_does_not_turn_green_sender_alive_led_off);
    return UNITY_END();
}
