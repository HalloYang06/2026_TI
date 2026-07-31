#ifndef MOTION_INTENT_H
#define MOTION_INTENT_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Hardware-independent line-following output. Wheel speeds keep the existing
 * competition unit of encoder counts per 100 ms; timestamp_ms identifies the
 * line snapshot that produced the request.
 */
typedef struct
{
    bool valid;
    uint32_t timestamp_ms;
    int16_t requested_speed_left;
    int16_t requested_speed_right;
    int16_t duty_slew_step;
} motion_intent_t;

#endif
