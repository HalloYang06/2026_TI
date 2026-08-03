#ifndef WIT_BYTE_QUEUE_H
#define WIT_BYTE_QUEUE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WIT_BYTE_QUEUE_CAPACITY 1024U

typedef struct
{
    uint8_t bytes[WIT_BYTE_QUEUE_CAPACITY];
    volatile uint16_t write_index;
    volatile uint16_t read_index;
    volatile uint32_t dropped;
    volatile uint16_t high_water;
} wit_byte_queue_t;

void wit_byte_queue_init(wit_byte_queue_t *queue);
uint16_t wit_byte_queue_push(
    wit_byte_queue_t *queue,
    const uint8_t *data,
    uint16_t length
);
uint16_t wit_byte_queue_pop(
    wit_byte_queue_t *queue,
    uint8_t *data,
    uint16_t capacity
);
uint16_t wit_byte_queue_depth(const wit_byte_queue_t *queue);
uint32_t wit_byte_queue_dropped(const wit_byte_queue_t *queue);
uint16_t wit_byte_queue_high_water(const wit_byte_queue_t *queue);

#ifdef __cplusplus
}
#endif

#endif
