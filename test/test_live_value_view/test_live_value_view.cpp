// Display classification of live values (fresh / stale / missing).
#include <unity.h>

#include "LiveValueView.h"

using DisplayLogic::LiveValueView;
using DisplayLogic::ValueState;
using LiveData::Metric;

void setUp() {}
void tearDown() {}

namespace {
LiveData::Snapshot snapshotWith(Metric metric, float value, uint8_t ageTenths) {
    LiveData::Snapshot snap;
    const int index = static_cast<int>(metric);
    snap.validMask = 1u << index;
    snap.values[index] = value;
    snap.ageTenths[index] = ageTenths;
    return snap;
}
} // namespace

void test_fresh_value() {
    LiveValueView view;
    view.apply(snapshotWith(Metric::Rpm, 2000.0f, 2), 10000);
    const auto v = view.get(Metric::Rpm, 10100);
    TEST_ASSERT_EQUAL(static_cast<int>(ValueState::Fresh), static_cast<int>(v.state));
    TEST_ASSERT_EQUAL_FLOAT(2000.0f, v.value);
    TEST_ASSERT_EQUAL_UINT32(300, v.ageMs);
}

void test_missed_update_keeps_value_as_stale_instead_of_blank() {
    LiveValueView view;
    view.apply(snapshotWith(Metric::Rpm, 2000.0f, 0), 10000);
    const auto v = view.get(Metric::Rpm, 13000);
    TEST_ASSERT_EQUAL(static_cast<int>(ValueState::Stale), static_cast<int>(v.state));
    TEST_ASSERT_TRUE(v.hasValue());
}

void test_slow_value_is_fresh_longer_than_fast_value() {
    LiveValueView view;
    view.apply(snapshotWith(Metric::OilTemp, 95.0f, 0), 0);
    TEST_ASSERT_EQUAL(static_cast<int>(ValueState::Fresh), static_cast<int>(view.get(Metric::OilTemp, 5000).state));
}

void test_very_old_value_is_missing() {
    LiveValueView view;
    view.apply(snapshotWith(Metric::Speed, 50.0f, 0), 0);
    TEST_ASSERT_EQUAL(static_cast<int>(ValueState::Missing),
                      static_cast<int>(view.get(Metric::Speed, LiveValueView::MaxStaleMs + 1).state));
}

void test_unsupported_and_never_received() {
    LiveValueView view;
    LiveData::Snapshot snap;
    snap.unsupportedMask = 1u << static_cast<int>(Metric::Maf);
    view.apply(snap, 0);
    TEST_ASSERT_EQUAL(static_cast<int>(ValueState::Unsupported), static_cast<int>(view.get(Metric::Maf, 0).state));
    TEST_ASSERT_EQUAL(static_cast<int>(ValueState::Missing), static_cast<int>(view.get(Metric::Rpm, 0).state));
}

void test_sender_age_is_taken_into_account() {
    LiveValueView view;
    // The sender says the value is already 1.4 s old.
    view.apply(snapshotWith(Metric::Rpm, 900.0f, 14), 5000);
    TEST_ASSERT_EQUAL(static_cast<int>(ValueState::Stale), static_cast<int>(view.get(Metric::Rpm, 5200).state));
}

void test_smoother_converges_independent_of_frame_rate() {
    DisplayLogic::Smoother fast(100.0f);
    DisplayLogic::Smoother slow(100.0f);
    fast.update(0.0f, 0);
    slow.update(0.0f, 0);
    float a = 0.0f;
    for (uint32_t t = 10; t <= 1000; t += 10) a = fast.update(1000.0f, t);
    const float b = slow.update(1000.0f, 1000);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 1000.0f, a);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 1000.0f, b);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_fresh_value);
    RUN_TEST(test_missed_update_keeps_value_as_stale_instead_of_blank);
    RUN_TEST(test_slow_value_is_fresh_longer_than_fast_value);
    RUN_TEST(test_very_old_value_is_missing);
    RUN_TEST(test_unsupported_and_never_received);
    RUN_TEST(test_sender_age_is_taken_into_account);
    RUN_TEST(test_smoother_converges_independent_of_frame_rate);
    return UNITY_END();
}
