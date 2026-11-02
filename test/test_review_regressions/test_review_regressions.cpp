#include <unity.h>
#include <limits>
#include "BootValidation.h"
#include "ConsumptionAccumulator.h"
#include "JsonEscape.h"
#include "LiveData.h"
#include "LiveValueView.h"
#include "UpdateSafety.h"
// common is excluded from native library discovery; exercise its real adapter.
#include "../../lib/common/PID_Converter.cpp"

void setUp() {}
void tearDown() {}

void test_expiry_survives_repeated_wire_snapshots_and_timer_wrap() {
    for (uint32_t start : {1000u, UINT32_MAX - 30000u}) {
        LiveData::LiveValueTable table;
        DisplayLogic::LiveValueView view;
        table.update(LiveData::Metric::Rpm, 1800, start);
        for (uint32_t elapsed = 0; elapsed <= 120000; elapsed += 100) {
            uint8_t bytes[LiveData::EncodedSnapshotSize];
            LiveData::Snapshot received;
            LiveData::encodeSnapshot(table.snapshot(start + elapsed), bytes, sizeof(bytes));
            TEST_ASSERT_TRUE(LiveData::decodeSnapshot(bytes, sizeof(bytes), received));
            view.apply(received, start + elapsed);
            if (elapsed > LiveData::MaxValueAgeMs) {
                TEST_ASSERT_FALSE(view.get(LiveData::Metric::Rpm, start + elapsed).hasValue());
            }
        }
        table.update(LiveData::Metric::Rpm, 1900, start + 120100);
        view.apply(table.snapshot(start + 120100), start + 120100);
        TEST_ASSERT_EQUAL_FLOAT(1900, view.get(LiveData::Metric::Rpm, start + 120100).value);
    }
}

void test_invalid_pid_keeps_validity_and_negative_temperatures_remain_valid() {
    const uint8_t data[] = {39};
    TEST_ASSERT_FALSE(convertPID(0x0c, data, 1).valid);
    const auto temperature = convertPID(0x05, data, 1);
    TEST_ASSERT_TRUE(temperature.valid);
    TEST_ASSERT_EQUAL_FLOAT(-1, temperature.value);
    LiveData::LiveValueTable table;
    table.update(LiveData::Metric::Rpm, std::numeric_limits<float>::quiet_NaN(), 0);
    TEST_ASSERT_FALSE(table.snapshot(0).valid(LiveData::Metric::Rpm));
}

void test_slow_pid_remains_fresh_until_next_poll_with_margin() {
    LiveData::LiveValueTable table;
    for (auto metric : {LiveData::Metric::CoolantTemp, LiveData::Metric::EcuVoltage}) {
        table.update(metric, 12, 1000);
        DisplayLogic::LiveValueView view;
        view.apply(table.snapshot(1000), 1000);
        TEST_ASSERT_EQUAL(static_cast<int>(DisplayLogic::ValueState::Fresh),
                          static_cast<int>(view.get(metric, 1000 + SenderConfig::SlowPidIntervalMs + 500).state));
    }
}

void test_json_preserves_quotes_backslashes_and_control_characters() {
    std::string input = "Test\"Net\\\n\r\t";
    input.push_back(0);
    input.push_back(1);
    const auto escaped = Json::escape(input);
    TEST_ASSERT_EQUAL_STRING("Test\\\"Net\\\\\\n\\r\\t\\u0000\\u0001", escaped.c_str());
    TEST_ASSERT_EQUAL_STRING("Grüße", Json::escape("Grüße").c_str());
}

void test_update_requires_fresh_parked_state_or_explicit_maintenance() {
    using namespace FirmwareUpdate;
    UpdateSafety gate;
    TEST_ASSERT_TRUE(gate.blocked(0));
    TEST_ASSERT_FALSE(gate.enableMaintenance(0, false));
    gate.observe(VehicleCondition::Parked, 100);
    TEST_ASSERT_FALSE(gate.blocked(100));
    TEST_ASSERT_TRUE(gate.blocked(100 + UpdateSafety::StatusFreshMs + 1));
    gate.observe(VehicleCondition::Active, 4000);
    TEST_ASSERT_TRUE(gate.blocked(4000));
    TEST_ASSERT_FALSE(gate.enableMaintenance(4000, true));
    TEST_ASSERT_TRUE(gate.blocked(8000));
    TEST_ASSERT_TRUE(gate.enableMaintenance(8000, true));
    TEST_ASSERT_FALSE(gate.blocked(8000));
    TEST_ASSERT_TRUE(gate.blocked(8000 + UpdateSafety::MaintenanceDurationMs));
    TEST_ASSERT_TRUE(gate.enableMaintenance(UINT32_MAX - 100, true));
    TEST_ASSERT_FALSE(gate.blocked(100));
    gate.observe(VehicleCondition::Active, 101);
    TEST_ASSERT_TRUE(gate.blocked(101));
    TEST_ASSERT_FALSE(gate.maintenance(101));
    TEST_ASSERT_EQUAL(static_cast<int>(VehicleCondition::Unknown), static_cast<int>(vehicleCondition("Booting")));
}

void test_boot_health_requires_continuous_local_success() {
    using namespace BootValidation;
    HealthGate gate;
    TEST_ASSERT_EQUAL(static_cast<int>(Decision::Wait), static_cast<int>(gate.evaluate(90000, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(Decision::Wait), static_cast<int>(gate.evaluate(119999, true)));
    TEST_ASSERT_EQUAL(static_cast<int>(Decision::Confirm), static_cast<int>(gate.evaluate(120000, true)));
    HealthGate failed;
    failed.evaluate(0, true);
    failed.evaluate(29000, false);
    TEST_ASSERT_EQUAL(static_cast<int>(Decision::Wait), static_cast<int>(failed.evaluate(30000, true)));
    failed.evaluate(59999, false);
    TEST_ASSERT_EQUAL(static_cast<int>(Decision::Rollback), static_cast<int>(failed.evaluate(60000, true)));
    HealthGate wrap;
    wrap.evaluate(UINT32_MAX - 100, true);
    TEST_ASSERT_EQUAL(static_cast<int>(Decision::Confirm), static_cast<int>(wrap.evaluate(30000, true)));
}

void test_consumption_weights_distance_and_accepts_zero_fuel_and_standstill() {
    Obd::ConsumptionAccumulator c;
    for (uint32_t t = 0; t <= 10000; t += 1000) {
        c.updateSpeed(t < 5000 ? 10 : 100, t);
        c.updateFuelRate(1, t);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 100.0 / 55.0, c.average());
    c.updateFuelRate(0, 10000);
    const float before = c.average();
    c.updateSpeed(100, 11000);
    c.updateFuelRate(0, 11000);
    TEST_ASSERT_TRUE(c.average() < before);
    TEST_ASSERT_EQUAL_FLOAT(0, c.instant());
    c.updateSpeed(0, 11000);
    c.updateFuelRate(1, 11000);
    const double litres = c.litres();
    const double distance = c.kilometres();
    c.advance(12000);
    TEST_ASSERT_TRUE(c.litres() > litres);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, distance, c.kilometres());
    TEST_ASSERT_FALSE(c.hasInstant(12000));
}

void test_consumption_stale_gap_does_not_count_and_frequency_does_not_change_average() {
    Obd::ConsumptionAccumulator fast, slow;
    for (uint32_t t = 0; t <= 10000; t += 100) {
        fast.updateSpeed(100, t);
        fast.updateFuelRate(5, t);
        if (t % 1000 == 0) {
            slow.updateSpeed(100, t);
            slow.updateFuelRate(5, t);
        }
    }
    TEST_ASSERT_FLOAT_WITHIN(0.00001, fast.average(), slow.average());
    slow.advance(20000);
    const double distance = slow.kilometres();
    slow.advance(60000);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, distance, slow.kilometres());
    TEST_ASSERT_FALSE(slow.fresh(60000));
    slow.updateSpeed(100, 60000);
    slow.updateFuelRate(5, 60000);
    TEST_ASSERT_FLOAT_WITHIN(0.000001, distance, slow.kilometres());
    slow.advance(61000);
    TEST_ASSERT_TRUE(slow.kilometres() > distance);
    slow.reset();
    TEST_ASSERT_FALSE(slow.hasAverage());
    TEST_ASSERT_FALSE(slow.fresh(61000));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_expiry_survives_repeated_wire_snapshots_and_timer_wrap);
    RUN_TEST(test_invalid_pid_keeps_validity_and_negative_temperatures_remain_valid);
    RUN_TEST(test_slow_pid_remains_fresh_until_next_poll_with_margin);
    RUN_TEST(test_json_preserves_quotes_backslashes_and_control_characters);
    RUN_TEST(test_update_requires_fresh_parked_state_or_explicit_maintenance);
    RUN_TEST(test_boot_health_requires_continuous_local_success);
    RUN_TEST(test_consumption_weights_distance_and_accepts_zero_fuel_and_standstill);
    RUN_TEST(test_consumption_stale_gap_does_not_count_and_frequency_does_not_change_average);
    return UNITY_END();
}
