

#include "gmock/gmock.h"

extern "C" {
    #include "queue.h"
}

using namespace testing;


TEST(basic_check, test_init_and_delete) {
    Queue* q = queue_init(5, 10);
    queue_delete(q);
    //todo: reliable way to measure allocated memory?
}


TEST(basic_check, test_fill_and_empty_queue) {
    Queue* q = queue_init(5, 3);
    byte temp[3] = {0u, 0u, 0u};


    for(byte i = 1; i <= 5; i++) {
        byte data[3] = {0u,0u, i};
        ASSERT_EQ(queue_push(q,data), SUCCESS);
    }
    ASSERT_EQ(queue_push(q,temp), ERROR);

    for(byte i = 1; i <= 5; i++) {
        byte data[3];
        ASSERT_EQ(queue_pop(q,data), SUCCESS);
        ASSERT_EQ(data[0], 0u);
        ASSERT_EQ(data[1], 0u);
        ASSERT_EQ(data[2], i);
    }
    ASSERT_EQ(queue_pop(q,temp), ERROR);
}



