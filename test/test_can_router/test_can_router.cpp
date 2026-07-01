#include <unity.h>
#include "CanRouter.h"

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

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_router_routes_to_all_registered_listeners);
    RUN_TEST(test_router_ignores_duplicate_registration);
    RUN_TEST(test_router_unregisters_listener);
    return UNITY_END();
}
