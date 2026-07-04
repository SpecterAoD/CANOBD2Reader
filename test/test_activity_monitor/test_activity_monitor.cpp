#include <unity.h>

#include "ActivityMonitor.h"
#include "config/PowerConfig.h"

namespace {

Power::ActivityInput baseInput(uint32_t nowMs = 10000) {
    Power::ActivityInput input{};
    input.nowMs = nowMs;
    input.bootCompletedAtMs = 1000;
    input.canDriverReady = true;
    input.obdEnabled = true;
    input.batteryVoltage = 12.5f;
    return input;
}

} // namespace

void setUp() {}
void tearDown() {}

void test_start_stop_is_detected_when_engine_is_off_but_bus_and_obd_are_active() {
    Power::ActivityMonitor monitor;
    monitor.reset(0);
    Power::ActivityInput input = baseInput();
    input.lastCanMessageAtMs = input.nowMs - 200;
    input.lastObdResponseAtMs = input.nowMs - 250;
    input.rpm = 0.0f;
    input.speedKph = 0.0f;

    const Power::ActivitySnapshot snapshot = monitor.update(input);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Power::VehicleState::StartStop),
                            static_cast<uint8_t>(snapshot.state));
    TEST_ASSERT_TRUE(snapshot.startStopDetected);
    TEST_ASSERT_FALSE(snapshot.parkedDetected);
    TEST_ASSERT_EQUAL_UINT8(12, snapshot.activityScore);
}

void test_engine_off_alone_does_not_request_display_sleep() {
    Power::ActivityMonitor monitor;
    monitor.reset(0);
    Power::ActivityInput input = baseInput();
    input.lastCanMessageAtMs = input.nowMs - 100;
    input.lastObdResponseAtMs = input.nowMs - 100;
    input.rpm = 0.0f;
    input.speedKph = 0.0f;

    const Power::ActivitySnapshot snapshot = monitor.update(input);

    TEST_ASSERT_NOT_EQUAL_UINT8(static_cast<uint8_t>(Power::VehicleState::DisplaySleep),
                                static_cast<uint8_t>(snapshot.state));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Power::PowerCommand::None),
                            static_cast<uint8_t>(snapshot.command));
}

void test_can_activity_prevents_parked_state() {
    Power::ActivityMonitor monitor;
    monitor.reset(0);
    Power::ActivityInput input = baseInput(PowerConfig::ParkDetectionTimeoutMs + 20000);
    input.lastCanMessageAtMs = input.nowMs - 500;
    input.lastObdResponseAtMs = input.nowMs - PowerConfig::ParkDetectionTimeoutMs - 1000;

    const Power::ActivitySnapshot snapshot = monitor.update(input);

    TEST_ASSERT_NOT_EQUAL_UINT8(static_cast<uint8_t>(Power::VehicleState::Parked),
                                static_cast<uint8_t>(snapshot.state));
    TEST_ASSERT_FALSE(snapshot.parkedDetected);
}

void test_parked_starts_sleep_timer_but_does_not_sleep_immediately() {
    Power::ActivityMonitor monitor;
    monitor.reset(0);
    Power::ActivityInput input = baseInput(PowerConfig::ParkDetectionTimeoutMs + 20000);
    input.batteryVoltage = 11.8f;
    input.lastCanMessageAtMs = 1000;
    input.lastObdResponseAtMs = 1000;

    const Power::ActivitySnapshot snapshot = monitor.update(input);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Power::VehicleState::Parked),
                            static_cast<uint8_t>(snapshot.state));
    TEST_ASSERT_TRUE(snapshot.parkedDetected);
    TEST_ASSERT_GREATER_THAN_UINT32(input.nowMs, snapshot.displaySleepDueAtMs);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Power::PowerCommand::None),
                            static_cast<uint8_t>(snapshot.command));
}

void test_display_sleep_after_parked_timer_elapsed() {
    Power::ActivityMonitor monitor;
    monitor.reset(0);
    Power::ActivityInput input = baseInput(PowerConfig::ParkDetectionTimeoutMs + 20000);
    input.batteryVoltage = 11.8f;
    input.lastCanMessageAtMs = 1000;
    input.lastObdResponseAtMs = 1000;
    monitor.update(input);

    input.nowMs += PowerConfig::DisplaySleepAfterMs + 1;
    const Power::ActivitySnapshot snapshot = monitor.update(input);

    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Power::VehicleState::DisplaySleep),
                            static_cast<uint8_t>(snapshot.state));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Power::PowerCommand::Sleep),
                            static_cast<uint8_t>(snapshot.command));
}

void test_wakeup_from_display_sleep_on_new_can_message() {
    Power::ActivityMonitor monitor;
    monitor.reset(0);
    Power::ActivityInput input = baseInput(PowerConfig::ParkDetectionTimeoutMs + 20000);
    input.batteryVoltage = 11.8f;
    input.lastCanMessageAtMs = 1000;
    input.lastObdResponseAtMs = 1000;
    monitor.update(input);

    input.nowMs += PowerConfig::DisplaySleepAfterMs + 1;
    monitor.update(input);

    input.nowMs += PowerConfig::WakeupDebounceMs + 1;
    input.lastCanMessageAtMs = input.nowMs;
    input.batteryVoltage = 12.5f;
    const Power::ActivitySnapshot snapshot = monitor.update(input);

    TEST_ASSERT_NOT_EQUAL_UINT8(static_cast<uint8_t>(Power::VehicleState::DisplaySleep),
                                static_cast<uint8_t>(snapshot.state));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Power::PowerCommand::Wakeup),
                            static_cast<uint8_t>(snapshot.command));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_start_stop_is_detected_when_engine_is_off_but_bus_and_obd_are_active);
    RUN_TEST(test_engine_off_alone_does_not_request_display_sleep);
    RUN_TEST(test_can_activity_prevents_parked_state);
    RUN_TEST(test_parked_starts_sleep_timer_but_does_not_sleep_immediately);
    RUN_TEST(test_display_sleep_after_parked_timer_elapsed);
    RUN_TEST(test_wakeup_from_display_sleep_on_new_can_message);
    return UNITY_END();
}
