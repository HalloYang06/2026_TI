#include "wit_byte_queue.h"

#include <stddef.h>

_Static_assert(
    (WIT_BYTE_QUEUE_CAPACITY & (WIT_BYTE_QUEUE_CAPACITY - 1U)) == 0U,
    "WIT byte queue capacity must be a power of two"
);

static uint16_t wit_byte_queue_clamped_depth(
    uint16_t write_index,
    uint16_t read_index
)
{
    const uint16_t depth = (uint16_t)(write_index - read_index);

    return (depth <= WIT_BYTE_QUEUE_CAPACITY)
        ? depth : WIT_BYTE_QUEUE_CAPACITY;
}

void wit_byte_queue_init(wit_byte_queue_t *queue)
{
    if (queue == NULL)
    {
        return;
    }
    queue->write_index = 0U;
    queue->read_index = 0U;
    queue->dropped = 0U;
    queue->high_water = 0U;
}

uint16_t wit_byte_queue_push(
    wit_byte_queue_t *queue,
    const uint8_t *data,
    uint16_t length
)
{
    uint16_t accepted = 0U;
    uint16_t write_index;
    uint16_t read_index;
    uint16_t depth;

    if ((queue == NULL) || (data == NULL))
    {
        return 0U;
    }
    write_index = queue->write_index;
    read_index = queue->read_index;
    depth = wit_byte_queue_clamped_depth(write_index, read_index);
    while ((accepted < length) && (depth < WIT_BYTE_QUEUE_CAPACITY))
    {
        queue->bytes[write_index & (WIT_BYTE_QUEUE_CAPACITY - 1U)] =
            data[accepted];
        write_index++;
        accepted++;
        depth++;
    }

    /* Publish only after all accepted bytes have been copied. */
    queue->write_index = write_index;
    if (depth > queue->high_water)
    {
        queue->high_water = depth;
    }
    queue->dropped += (uint32_t)(length - accepted);
    return accepted;
}

uint16_t wit_byte_queue_pop(
    wit_byte_queue_t *queue,
    uint8_t *data,
    uint16_t capacity
)
{
    uint16_t read_index;
    uint16_t available;
    uint16_t count;

    if ((queue == NULL) || (data == NULL))
    {
        return 0U;
    }
    read_index = queue->read_index;
    available = wit_byte_queue_clamped_depth(
        queue->write_index, read_index
    );
    count = (available < capacity) ? available : capacity;
    for (uint16_t i = 0U; i < count; ++i)
    {
        data[i] = queue->bytes[
            read_index & (WIT_BYTE_QUEUE_CAPACITY - 1U)
        ];
        read_index++;
    }

    /* The ISR producer never modifies read_index. */
    queue->read_index = read_index;
    return count;
}

uint16_t wit_byte_queue_depth(const wit_byte_queue_t *queue)
{
    if (queue == NULL)
    {
        return 0U;
    }
    return wit_byte_queue_clamped_depth(
        queue->write_index, queue->read_index
    );
}

uint32_t wit_byte_queue_dropped(const wit_byte_queue_t *queue)
{
    return (queue != NULL) ? queue->dropped : 0U;
}

uint16_t wit_byte_queue_high_water(const wit_byte_queue_t *queue)
{
    return (queue != NULL) ? queue->high_water : 0U;
}
