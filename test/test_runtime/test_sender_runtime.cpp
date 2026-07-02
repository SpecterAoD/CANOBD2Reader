#include <unity.h>
#include "SenderLoopState.h"
#include "SenderRuntimeCoordinator.h"
#include "SenderRuntimeState.h"

namespace {

constexpr uint32_t kTestPollingRateMs = 25;
constexpr uint32_t kTestTwaiLogIntervalMs = 100;

struct CoordinatorFixture {
    bool simulationEnabled = false;
    bool senderStarted = true;
    Runtime::SenderRuntimeCoordinator::CanAlertResult canAlert{};
    int otaCalls = 0;
    int webCalls = 0;
    int webStatusCalls = 0;
    int ledTestCalls = 0;
    int heartbeatCalls = 0;
    int simulationTickCalls = 0;
    int canAlertCalls = 0;
    int setLastErrorCalls = 0;
    int pulseErrorCalls = 0;
    int updateLedCalls = 0;
    int obdCalls = 0;
    int udsCalls = 0;
    int twaiLogCalls = 0;
    int powerCalls = 0;
    uint32_t lastSimulationTickAt = 0;
    uint32_t lastPowerTickAt = 0;
    const char* lastErrorText = nullptr;
};

CoordinatorFixture g_coordinatorFixture;

void handleOta() { ++g_coordinatorFixture.otaCalls; }
void handleWeb() { ++g_coordinatorFixture.webCalls; }
void updateWebStatus(const Runtime::SenderLoopState&) { ++g_coordinatorFixture.webStatusCalls; }
void updateLedTestButton() { ++g_coordinatorFixture.ledTestCalls; }
void sendHeartbeat(Runtime::SenderLoopState&) { ++g_coordinatorFixture.heartbeatCalls; }
bool simulationEnabled() { return g_coordinatorFixture.simulationEnabled; }
void tickSimulation(uint32_t nowMs) {
    ++g_coordinatorFixture.simulationTickCalls;
    g_coordinatorFixture.lastSimulationTickAt = nowMs;
}
bool senderStarted() { return g_coordinatorFixture.senderStarted; }
Runtime::SenderRuntimeCoordinator::CanAlertResult processCanAlerts(uint32_t) {
    ++g_coordinatorFixture.canAlertCalls;
    return g_coordinatorFixture.canAlert;
}
void setLastError(const char* errorText) {
    ++g_coordinatorFixture.setLastErrorCalls;
    g_coordinatorFixture.lastErrorText = errorText;
}
void pulseErrorLed(uint32_t) { ++g_coordinatorFixture.pulseErrorCalls; }
void updateLed(uint32_t) { ++g_coordinatorFixture.updateLedCalls; }
void tickObd(Runtime::SenderLoopState&) { ++g_coordinatorFixture.obdCalls; }
void tickUds(Runtime::SenderLoopState&) { ++g_coordinatorFixture.udsCalls; }
void logTwaiStatus() { ++g_coordinatorFixture.twaiLogCalls; }
void tickPower(uint32_t nowMs) {
    ++g_coordinatorFixture.powerCalls;
    g_coordinatorFixture.lastPowerTickAt = nowMs;
}

Runtime::SenderRuntimeCoordinator makeCoordinator() {
    return Runtime::SenderRuntimeCoordinator(
        {kTestPollingRateMs, kTestTwaiLogIntervalMs, true},
        {handleOta,
         handleWeb,
         updateWebStatus,
         updateLedTestButton,
         sendHeartbeat,
         simulationEnabled,
         tickSimulation,
         senderStarted,
         processCanAlerts,
         setLastError,
         pulseErrorLed,
         updateLed,
         tickObd,
         tickUds,
         logTwaiStatus,
         tickPower});
}

} // namespace

void setUp() {
    Runtime::SenderRuntimeState::reset();
    g_coordinatorFixture = {};
}

void tearDown() {}

void test_consumption_requires_speed_and_fuel_rate() {
    TEST_ASSERT_FALSE(Runtime::SenderRuntimeState::hasConsumptionInput());
    Runtime::SenderRuntimeState::updateSpeed(50.0f);
    TEST_ASSERT_FALSE(Runtime::SenderRuntimeState::hasConsumptionInput());
    Runtime::SenderRuntimeState::updateFuelRate(5.0f);
    TEST_ASSERT_TRUE(Runtime::SenderRuntimeState::hasConsumptionInput());
}

void test_consumption_average_after_ten_samples() {
    Runtime::SenderRuntimeState::updateSpeed(100.0f);
    Runtime::SenderRuntimeState::updateFuelRate(7.5f);

    float average = 0.0f;
    for (int i = 0; i < 9; ++i) {
        TEST_ASSERT_FALSE(Runtime::SenderRuntimeState::addConsumptionSample(average));
    }

    TEST_ASSERT_TRUE(Runtime::SenderRuntimeState::addConsumptionSample(average));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.5f, average);
    TEST_ASSERT_EQUAL_UINT32(0, Runtime::SenderRuntimeState::consumptionSampleCount());
}

void test_sender_loop_state_tracks_can_activity() {
    Runtime::SenderLoopState state;
    state.resetForBoot(1000, 250);

    TEST_ASSERT_FALSE(state.canBusActive);
    TEST_ASSERT_FALSE(state.canRecent(1000, 200));

    state.markCanTraffic(1250);
    TEST_ASSERT_TRUE(state.canBusActive);
    TEST_ASSERT_TRUE(state.canRecent(1300, 100));
    TEST_ASSERT_FALSE(state.canRecent(1400, 100));
}

void test_sender_loop_state_resets_timestamps() {
    Runtime::SenderLoopState state;
    state.markCanTraffic(500);
    state.heartbeatCount = 3;
    state.canDriverReady = true;
    state.espNowReady = true;

    state.resetForBoot(50, 200);

    TEST_ASSERT_EQUAL_UINT32(0, state.lastCanMessageAt);
    TEST_ASSERT_EQUAL_UINT32(0, state.lastHeartbeatSentAt);
    TEST_ASSERT_EQUAL_UINT32(0, state.lastTwaiStatusLogAt);
    TEST_ASSERT_EQUAL_UINT32(0, state.heartbeatCount);
    TEST_ASSERT_FALSE(state.canBusActive);
    TEST_ASSERT_FALSE(state.canDriverReady);
    TEST_ASSERT_FALSE(state.espNowReady);
}

void test_runtime_coordinator_runs_simulation_path_before_schedulers() {
    auto coordinator = makeCoordinator();
    coordinator.resetForBoot(10, 250, true, true);
    g_coordinatorFixture.simulationEnabled = true;

    coordinator.tick(125);

    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.otaCalls);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.webCalls);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.webStatusCalls);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.ledTestCalls);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.heartbeatCalls);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.simulationTickCalls);
    TEST_ASSERT_EQUAL_UINT32(125, g_coordinatorFixture.lastSimulationTickAt);
    TEST_ASSERT_EQUAL_INT(0, g_coordinatorFixture.canAlertCalls);
    TEST_ASSERT_EQUAL_INT(0, g_coordinatorFixture.powerCalls);
}

void test_runtime_coordinator_processes_can_alerts_and_schedulers() {
    auto coordinator = makeCoordinator();
    coordinator.resetForBoot(0, 250, true, true);
    g_coordinatorFixture.canAlert = {true, true, "CAN bus error"};

    coordinator.tick(220);

    TEST_ASSERT_EQUAL_UINT32(220, coordinator.state().currentMillis);
    TEST_ASSERT_TRUE(coordinator.state().canBusActive);
    TEST_ASSERT_EQUAL_UINT32(220, coordinator.state().lastCanMessageAt);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.canAlertCalls);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.setLastErrorCalls);
    TEST_ASSERT_EQUAL_STRING("CAN bus error", g_coordinatorFixture.lastErrorText);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.pulseErrorCalls);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.updateLedCalls);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.obdCalls);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.udsCalls);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.twaiLogCalls);
    TEST_ASSERT_EQUAL_INT(1, g_coordinatorFixture.powerCalls);
    TEST_ASSERT_EQUAL_UINT32(220, g_coordinatorFixture.lastPowerTickAt);
}

void test_runtime_coordinator_respects_start_and_driver_guards() {
    auto coordinator = makeCoordinator();
    coordinator.resetForBoot(0, 250, true, false);
    g_coordinatorFixture.senderStarted = false;

    coordinator.tick(50);

    TEST_ASSERT_EQUAL_INT(0, g_coordinatorFixture.canAlertCalls);
    TEST_ASSERT_EQUAL_INT(0, g_coordinatorFixture.obdCalls);
    TEST_ASSERT_EQUAL_INT(0, g_coordinatorFixture.udsCalls);
    TEST_ASSERT_EQUAL_INT(0, g_coordinatorFixture.powerCalls);
    TEST_ASSERT_EQUAL_INT(0, g_coordinatorFixture.twaiLogCalls);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_consumption_requires_speed_and_fuel_rate);
    RUN_TEST(test_consumption_average_after_ten_samples);
    RUN_TEST(test_sender_loop_state_tracks_can_activity);
    RUN_TEST(test_sender_loop_state_resets_timestamps);
    RUN_TEST(test_runtime_coordinator_runs_simulation_path_before_schedulers);
    RUN_TEST(test_runtime_coordinator_processes_can_alerts_and_schedulers);
    RUN_TEST(test_runtime_coordinator_respects_start_and_driver_guards);
    return UNITY_END();
}
