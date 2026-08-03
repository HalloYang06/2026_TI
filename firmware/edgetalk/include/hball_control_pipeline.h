#ifndef HBALL_CONTROL_PIPELINE_H
#define HBALL_CONTROL_PIPELINE_H

#include "hball_deployment_controller.h"
#include "hball_fourbar.h"
#include "hball_sensor_fusion.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_CONTROL_MIN_VISION_CONFIDENCE 0.40F
#define HBALL_CONTROL_TRACKING_MAX_AGE_MS 25U
#define HBALL_CONTROL_COASTING_MAX_AGE_MS 50U
#define HBALL_CONTROL_DEGRADED_MAX_AGE_MS 100U

typedef enum
{
    HBALL_CONTROL_WAITING_FOR_VISION = 0,
    HBALL_CONTROL_TRACKING,
    HBALL_CONTROL_COASTING,
    HBALL_CONTROL_DEGRADED,
    HBALL_CONTROL_VISION_LOST,
} hball_control_mode_t;

typedef struct
{
    hball_control_mode_t mode;
    bool safety_eligible;
    float shadow_command_rad;
    float estimated_position_m;
    float estimated_velocity_mps;
    float estimated_disturbance_mps2;
    float actual_pipe_angle_rad;
    float motor_target_rad;
    bool linkage_valid;
    bool linkage_calibrated;
} hball_control_output_t;

typedef struct
{
    hball_deployment_controller_t controller;
    hball_fourbar_geometry_t fourbar;
    float motor_level_angle_rad;
    uint32_t last_vision_sequence;
    uint32_t step_total;
    uint32_t vision_measurements_consumed;
    uint32_t duplicate_vision_skips;
    uint32_t low_confidence_vision_skips;
    uint32_t rejected_vision_measurements;
    uint32_t vision_relocks;
    uint8_t consecutive_vision_rejects;
    bool vision_sequence_initialized;
    bool linkage_calibrated;
} hball_control_pipeline_t;

void hball_control_pipeline_init(
    hball_control_pipeline_t *pipeline, float initial_position_m
);
bool hball_control_pipeline_set_motor_level(
    hball_control_pipeline_t *pipeline, float motor_level_angle_rad
);
void hball_control_pipeline_step(
    hball_control_pipeline_t *pipeline,
    const hball_sensor_snapshot_t *snapshot,
    float dt_s,
    float target_position_m,
    hball_control_output_t *output
);

#ifdef __cplusplus
}
#endif

#endif
