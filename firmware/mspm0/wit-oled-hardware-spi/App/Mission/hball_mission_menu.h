#ifndef HBALL_MISSION_MENU_H
#define HBALL_MISSION_MENU_H

#include "hball_mission_client.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    HBALL_MISSION_MENU_NONE = 0,
    HBALL_MISSION_MENU_SELECT,
    HBALL_MISSION_MENU_EXECUTE
} hball_mission_menu_event_t;

typedef enum
{
    HBALL_MISSION_MENU_NO_CHANGE = 0,
    HBALL_MISSION_MENU_SELECTED,
    HBALL_MISSION_MENU_LOCAL_START_ACCEPTED,
    HBALL_MISSION_MENU_START_ACCEPTED,
    HBALL_MISSION_MENU_START_BLOCKED,
    HBALL_MISSION_MENU_LOCKED
} hball_mission_menu_result_t;

typedef struct
{
    const char *mission_label;
    const char *state_label;
    const char *missing_label;
    uint16_t epoch;
    uint16_t ready_mask;
    uint8_t mission_id;
    uint8_t global_state;
    int16_t motor_angle_mrad;
    int16_t target_position_mm;
    uint8_t setup_flags;
    bool local_execution;
    bool status_fresh;
    bool start_requested;
    bool setup_valid;
} hball_mission_menu_view_t;

uint8_t hball_mission_menu_next(uint8_t mission_id);
hball_mission_menu_result_t hball_mission_menu_handle(
    hball_mission_client_t *client,
    hball_mission_menu_event_t event,
    uint32_t now_ms
);
uint16_t hball_mission_menu_required_mask(uint8_t mission_id);
const char *hball_mission_menu_mission_label(uint8_t mission_id);
const char *hball_mission_menu_state_label(uint8_t global_state);
const char *hball_mission_menu_missing_label(
    const hball_mission_client_t *client, uint32_t now_ms
);
bool hball_mission_menu_make_view(
    const hball_mission_client_t *client,
    uint32_t now_ms,
    hball_mission_menu_view_t *view
);
bool hball_mission_menu_view_equal(
    const hball_mission_menu_view_t *left,
    const hball_mission_menu_view_t *right
);

#ifdef __cplusplus
}
#endif

#endif
