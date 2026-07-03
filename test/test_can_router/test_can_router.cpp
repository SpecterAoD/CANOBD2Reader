#include <unity.h>
#include "CanRouter.h"
#include "CanRouterHub.h"

namespace {
int pumpCalls = 0;

void testPump() {
    ++pumpCalls;
}
}

class CountingListener final : public CanRouting::CanFrameListener {
public:
    void onCanFrame(const CanRouting::CanFrame& frame) override {
        ++count;
        lastId = frame.id;
        lastLength = frame.length;
    }

    int count = 0;
    uint32_t lastId = 0;
    uint8_t lastLength = 0;
};

void setUp() {}
void tearDown() {}

void test_router_routes_to_all_registered_listeners() {
    CanRouting::CanRouter router;
    CountingListener first;
    CountingListener second;

    TEST_ASSERT_TRUE(router.registerListener(first));
    TEST_ASSERT_TRUE(router.registerListener(second));
    TEST_ASSERT_EQUAL_UINT(2, router.listenerCount());

    CanRouting::CanFrame frame{};
    frame.id = 0x7E8;
    frame.length = 8;
    router.route(frame);

    TEST_ASSERT_EQUAL_INT(1, first.count);
    TEST_ASSERT_EQUAL_INT(1, second.count);
    TEST_ASSERT_EQUAL_UINT32(0x7E8, first.lastId);
    TEST_ASSERT_EQUAL_UINT8(8, second.lastLength);
}

void test_router_ignores_duplicate_registration() {
    CanRouting::CanRouter router;
    CountingListener listener;

    TEST_ASSERT_TRUE(router.registerListener(listener));
    TEST_ASSERT_TRUE(router.registerListener(listener));
    TEST_ASSERT_EQUAL_UINT(1, router.listenerCount());
}

void test_router_unregisters_listener() {
    CanRouting::CanRouter router;
    CountingListener first;
    CountingListener second;

    TEST_ASSERT_TRUE(router.registerListener(first));
    TEST_ASSERT_TRUE(router.registerListener(second));
    TEST_ASSERT_TRUE(router.unregisterListener(first));
    TEST_ASSERT_EQUAL_UINT(1, router.listenerCount());

    CanRouting::CanFrame frame{};
    router.route(frame);

    TEST_ASSERT_EQUAL_INT(0, first.count);
    TEST_ASSERT_EQUAL_INT(1, second.count);
}

void test_shared_router_and_pump() {
    CanRouting::clearListeners();
    pumpCalls = 0;
    CountingListener listener;
    TEST_ASSERT_TRUE(CanRouting::registerListener(listener));

    CanRouting::CanFrame frame{};
    frame.id = 0x321;
    frame.length = 2;
    CanRouting::routeFrame(frame);
    TEST_ASSERT_EQUAL_INT(1, listener.count);
    TEST_ASSERT_EQUAL_UINT32(0x321, listener.lastId);

    CanRouting::setFramePump(testPump);
    CanRouting::pumpFrames();
    TEST_ASSERT_EQUAL_INT(1, pumpCalls);

    TEST_ASSERT_TRUE(CanRouting::unregisterListener(listener));
    CanRouting::setFramePump(nullptr);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_router_routes_to_all_registered_listeners);
    RUN_TEST(test_router_ignores_duplicate_registration);
    RUN_TEST(test_router_unregisters_listener);
    RUN_TEST(test_shared_router_and_pump);
    return UNITY_END();
}
