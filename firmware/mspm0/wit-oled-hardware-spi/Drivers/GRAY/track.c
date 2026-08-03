#include "track.h"
#include "line_sensor_port.h"

#define TRACK_UPDATE_PERIOD_MS   10U
#define TRACK_FORWARD_TIME_MS   200U
#define TRACK_TURN_TIME_MS      800U
#define TRACK_MARKER_LOCK_MS    500U

#define TRACK_SENSOR(index)      (1U << (index))

typedef enum {
    TRACK_STATE_FOLLOW = 0,
    TRACK_STATE_FORWARD,
    TRACK_STATE_TURN_LEFT
} TrackState_t;

static TrackState_t track_state = TRACK_STATE_FOLLOW;
static uint32_t last_update_ms = 0;
static uint32_t state_start_ms = 0;
static uint32_t last_marker_ms = 0;
static uint16_t marker_count = 0;
static uint16_t marker_goal = 4;
static uint8_t marker_armed = 0;
static int8_t last_line_side = 0;

static void follow_line(uint8_t sensors)
{
    if (sensors == 0U)
    {
        /* 丢线后按照最后看到线的方向寻找，避免永远只向左转。 */
        if (last_line_side > 0) {
            chassis_actuator_set_pwm(40, -40);
        } else {
            chassis_actuator_set_pwm(-40, 40);
        }
    }
    else if ((sensors & TRACK_SENSOR(0)) != 0U)
    {
        chassis_actuator_set_pwm(-55, -10);
        last_line_side = -1;
    }
    else if ((sensors & TRACK_SENSOR(1)) != 0U)
    {
        chassis_actuator_set_pwm(-45, -15);
        last_line_side = -1;
    }
    else if ((sensors & TRACK_SENSOR(2)) != 0U)
    {
        chassis_actuator_set_pwm(-35, -15);
        last_line_side = -1;
    }
    else if ((sensors & TRACK_SENSOR(3)) != 0U)
    {
        chassis_actuator_set_pwm(-25, -15);
        last_line_side = -1;
    }
    else if ((sensors & TRACK_SENSOR(4)) != 0U)
    {
        chassis_actuator_set_pwm(-15, -25);
        last_line_side = 1;
    }
    else if ((sensors & TRACK_SENSOR(5)) != 0U)
    {
        chassis_actuator_set_pwm(-15, -35);
        last_line_side = 1;
    }
    else if ((sensors & TRACK_SENSOR(6)) != 0U)
    {
        chassis_actuator_set_pwm(-15, -45);
        last_line_side = 1;
    }
    else
    {
        chassis_actuator_set_pwm(-10, -55);
        last_line_side = 1;
    }
}

void track_start(uint8_t rounds)
{
    if (rounds < 1U) {
        rounds = 1U;
    } else if (rounds > 5U) {
        rounds = 5U;
    }

    marker_goal = (uint16_t)rounds * 4U;
    marker_count = 0;
    marker_armed = 0;
    last_line_side = 0;
    track_state = TRACK_STATE_FOLLOW;
    last_update_ms = tick_ms;
    state_start_ms = tick_ms;
    last_marker_ms = tick_ms;
}

void track_stop(void)
{
    chassis_actuator_stop();
    track_state = TRACK_STATE_FOLLOW;
    marker_armed = 0;
    start = 0;
}

void my_track(void)
{
    uint32_t now = tick_ms;
    uint8_t sensors;
    uint8_t on_turn_marker;

    if ((uint32_t)(now - last_update_ms) < TRACK_UPDATE_PERIOD_MS) {
        return;
    }
    last_update_ms = now;

    if (track_state == TRACK_STATE_FORWARD)
    {
        chassis_actuator_set_pwm(-15, -15);
        if ((uint32_t)(now - state_start_ms) >= TRACK_FORWARD_TIME_MS)
        {
            track_state = TRACK_STATE_TURN_LEFT;
            state_start_ms = now;
        }
        return;
    }

    if (track_state == TRACK_STATE_TURN_LEFT)
    {
        chassis_actuator_set_pwm(-70, 70);
        if ((uint32_t)(now - state_start_ms) >= TRACK_TURN_TIME_MS)
        {
            track_state = TRACK_STATE_FOLLOW;
            state_start_ms = now;
        }
        return;
    }

    sensors = line_sensor_port_read_active_mask();
    on_turn_marker = ((sensors & TRACK_SENSOR(0)) != 0U) &&
                     ((sensors & TRACK_SENSOR(3)) != 0U);

    if (on_turn_marker != 0U)
    {
        if ((marker_armed != 0U) &&
            ((uint32_t)(now - last_marker_ms) >= TRACK_MARKER_LOCK_MS))
        {
            marker_count++;
            marker_armed = 0;
            last_marker_ms = now;
            beep();
            flash();

            if (marker_count >= marker_goal)
            {
                track_stop();
                return;
            }

            track_state = TRACK_STATE_FORWARD;
            state_start_ms = tick_ms;
        }
        else
        {
            chassis_actuator_set_pwm(-20, -20);
        }
        return;
    }

    if ((uint32_t)(now - last_marker_ms) >= TRACK_MARKER_LOCK_MS) {
        marker_armed = 1;
    }

    follow_line(sensors);
}
