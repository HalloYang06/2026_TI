#include "wit_byte_queue.h"

#include <assert.h>
#include <stdint.h>

static void test_fifo_order_and_depth(void)
{
    wit_byte_queue_t queue;
    const uint8_t input[] = {0x55U, 0x51U, 0x01U, 0x02U};
    uint8_t output[sizeof(input)] = {0U};

    wit_byte_queue_init(&queue);
    assert(wit_byte_queue_push(&queue, input, sizeof(input)) == sizeof(input));
    assert(wit_byte_queue_depth(&queue) == sizeof(input));
    assert(wit_byte_queue_pop(&queue, output, 2U) == 2U);
    assert(output[0] == 0x55U);
    assert(output[1] == 0x51U);
    assert(wit_byte_queue_depth(&queue) == 2U);
    assert(wit_byte_queue_pop(&queue, output, sizeof(output)) == 2U);
    assert(output[0] == 0x01U);
    assert(output[1] == 0x02U);
    assert(wit_byte_queue_depth(&queue) == 0U);
}

static void test_full_queue_drops_new_bytes_and_records_high_water(void)
{
    wit_byte_queue_t queue;
    uint8_t input[WIT_BYTE_QUEUE_CAPACITY + 3U];
    uint8_t output[WIT_BYTE_QUEUE_CAPACITY];

    for (uint16_t i = 0U; i < sizeof(input); ++i)
    {
        input[i] = (uint8_t)i;
    }
    wit_byte_queue_init(&queue);
    assert(wit_byte_queue_push(&queue, input, sizeof(input))
           == WIT_BYTE_QUEUE_CAPACITY);
    assert(wit_byte_queue_depth(&queue) == WIT_BYTE_QUEUE_CAPACITY);
    assert(wit_byte_queue_high_water(&queue) == WIT_BYTE_QUEUE_CAPACITY);
    assert(wit_byte_queue_dropped(&queue) == 3U);
    assert(wit_byte_queue_pop(&queue, output, sizeof(output))
           == WIT_BYTE_QUEUE_CAPACITY);
    for (uint16_t i = 0U; i < sizeof(output); ++i)
    {
        assert(output[i] == (uint8_t)i);
    }
}

static void test_indices_wrap_without_reordering(void)
{
    wit_byte_queue_t queue;
    uint8_t input[17U];
    uint8_t output[17U];

    wit_byte_queue_init(&queue);
    for (uint32_t cycle = 0U; cycle < 5000U; ++cycle)
    {
        for (uint16_t i = 0U; i < sizeof(input); ++i)
        {
            input[i] = (uint8_t)(cycle + i);
        }
        assert(wit_byte_queue_push(&queue, input, sizeof(input))
               == sizeof(input));
        assert(wit_byte_queue_pop(&queue, output, sizeof(output))
               == sizeof(output));
        for (uint16_t i = 0U; i < sizeof(output); ++i)
        {
            assert(output[i] == input[i]);
        }
    }
    assert(wit_byte_queue_depth(&queue) == 0U);
    assert(wit_byte_queue_dropped(&queue) == 0U);
}

static void test_invalid_arguments_fail_closed(void)
{
    wit_byte_queue_t queue;
    uint8_t byte = 0x55U;

    wit_byte_queue_init(&queue);
    wit_byte_queue_init(NULL);
    assert(wit_byte_queue_push(NULL, &byte, 1U) == 0U);
    assert(wit_byte_queue_push(&queue, NULL, 1U) == 0U);
    assert(wit_byte_queue_pop(NULL, &byte, 1U) == 0U);
    assert(wit_byte_queue_pop(&queue, NULL, 1U) == 0U);
    assert(wit_byte_queue_depth(NULL) == 0U);
    assert(wit_byte_queue_dropped(NULL) == 0U);
    assert(wit_byte_queue_high_water(NULL) == 0U);
}

int main(void)
{
    test_fifo_order_and_depth();
    test_full_queue_drops_new_bytes_and_records_high_water();
    test_indices_wrap_without_reordering();
    test_invalid_arguments_fail_closed();
    return 0;
}
