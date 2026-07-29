#include "hball_m55_freertos.h"

#include "FreeRTOS.h"
#include "task.h"

#include "hball_control_pipeline.h"
#include "hball_m55_ipc.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define HBALL_M55_PERIOD_MS 5U
#define HBALL_M55_UI_PERIOD_MS 100U
#define HBALL_M55_TASK_STACK_WORDS 2048U
#define HBALL_M55_TASK_PRIORITY (configMAX_PRIORITIES - 2U)

static hball_control_pipeline_t g_hball_m55_pipeline;
static hball_control_output_t g_hball_m55_output;
static hball_sensor_snapshot_t g_hball_m55_snapshot;
static hball_m55_ui_snapshot_t g_hball_m55_ui_snapshot;
static TaskHandle_t g_hball_m55_task;
static uint32_t g_hball_m55_steps;
static uint32_t g_hball_m55_deadline_misses;
static uint32_t g_hball_m55_input_updates;

/* SAFETY: this task only publishes SHADOW_ONLY frames; ACTUATOR_TX=0. */

__attribute__((weak)) void hball_m55_platform_ui_10hz(
    const hball_m55_ui_snapshot_t *snapshot
)
{
    (void)snapshot;
}

static uint32_t hball_m55_now_ms(void)
{
    const uint64_t ticks = (uint64_t)xTaskGetTickCount();

    return (uint32_t)((ticks * UINT64_C(1000)) / configTICK_RATE_HZ);
}

static uint32_t hball_m55_add_age(uint32_t age_ms)
{
    return age_ms > (UINT32_MAX - HBALL_M55_PERIOD_MS)
        ? UINT32_MAX
        : age_ms + HBALL_M55_PERIOD_MS;
}

static void hball_m55_age_snapshot(void)
{
    g_hball_m55_snapshot.vision_receive_age_ms = hball_m55_add_age(
        g_hball_m55_snapshot.vision_receive_age_ms
    );
    g_hball_m55_snapshot.imu_age_ms = hball_m55_add_age(
        g_hball_m55_snapshot.imu_age_ms
    );
    g_hball_m55_snapshot.wheel_age_ms = hball_m55_add_age(
        g_hball_m55_snapshot.wheel_age_ms
    );
    g_hball_m55_snapshot.motor_age_ms = hball_m55_add_age(
        g_hball_m55_snapshot.motor_age_ms
    );
    g_hball_m55_snapshot.heartbeat_age_ms = hball_m55_add_age(
        g_hball_m55_snapshot.heartbeat_age_ms
    );
    if (g_hball_m55_snapshot.vision_receive_age_ms
        > HBALL_SENSOR_VISION_STALE_MS)
    {
        g_hball_m55_snapshot.valid_flags &= ~HBALL_SENSOR_VALID_VISION;
    }
    if (g_hball_m55_snapshot.imu_age_ms > HBALL_SENSOR_IMU_STALE_MS)
    {
        g_hball_m55_snapshot.valid_flags &= ~HBALL_SENSOR_VALID_IMU;
    }
    if (g_hball_m55_snapshot.wheel_age_ms > HBALL_SENSOR_WHEEL_STALE_MS)
    {
        g_hball_m55_snapshot.valid_flags &= ~HBALL_SENSOR_VALID_WHEEL;
    }
    if (g_hball_m55_snapshot.motor_age_ms > HBALL_SENSOR_MOTOR_STALE_MS)
    {
        g_hball_m55_snapshot.valid_flags &= ~HBALL_SENSOR_VALID_MOTOR;
    }
    if (g_hball_m55_snapshot.heartbeat_age_ms
        > HBALL_SENSOR_HEARTBEAT_STALE_MS)
    {
        g_hball_m55_snapshot.valid_flags &= ~HBALL_SENSOR_VALID_HEARTBEAT;
    }
}

static void hball_m55_make_ui_snapshot(hball_m55_ui_snapshot_t *ui)
{
    memset(ui, 0, sizeof(*ui));
    ui->controller_steps = g_hball_m55_steps;
    ui->deadline_misses = g_hball_m55_deadline_misses;
    ui->vision_age_ms = g_hball_m55_snapshot.vision_receive_age_ms;
    ui->ball_position_m = g_hball_m55_output.estimated_position_m;
    ui->ball_velocity_mps = g_hball_m55_output.estimated_velocity_mps;
    ui->longitudinal_accel_mps2 =
        g_hball_m55_snapshot.longitudinal_accel_mps2;
    ui->yaw_rate_rad_s = g_hball_m55_snapshot.yaw_rate_rad_s;
    ui->motor_angle_rad = g_hball_m55_snapshot.motor_angle_rad;
    ui->motor_velocity_rad_s = g_hball_m55_snapshot.motor_velocity_rad_s;
    ui->lqg_target_rad = g_hball_m55_output.shadow_command_rad;
    if ((g_hball_m55_snapshot.valid_flags & HBALL_SENSOR_VALID_IMU) != 0U)
    {
        ui->valid_flags |= HBALL_UI_VALID_IMU;
    }
    if ((g_hball_m55_snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) != 0U)
    {
        ui->valid_flags |= HBALL_UI_VALID_VISION;
    }
    if ((g_hball_m55_snapshot.valid_flags & HBALL_SENSOR_VALID_MOTOR) != 0U)
    {
        ui->valid_flags |= HBALL_UI_VALID_MOTOR;
    }
}

void hball_m55_get_ui_snapshot(hball_m55_ui_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }
    taskENTER_CRITICAL();
    *snapshot = g_hball_m55_ui_snapshot;
    taskEXIT_CRITICAL();
}

static void hball_m55_publish_shadow(uint32_t now_ms)
{
    hball_control_shadow_t shadow;

    memset(&shadow, 0, sizeof(shadow));
    shadow.source_sensor_sequence = g_hball_m55_snapshot.sequence;
    shadow.controller_steps = g_hball_m55_steps;
    shadow.deadline_misses = g_hball_m55_deadline_misses;
    shadow.produced_time_ms = now_ms;
    shadow.mode = (uint16_t)g_hball_m55_output.mode;
    shadow.flags = HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY;
    if (g_hball_m55_output.safety_eligible)
    {
        shadow.flags |= HBALL_IPC_CONTROL_FLAG_SAFETY_ELIGIBLE;
    }
    shadow.target_angle_rad = g_hball_m55_output.shadow_command_rad;
    shadow.estimated_position_m = g_hball_m55_output.estimated_position_m;
    shadow.estimated_velocity_mps = g_hball_m55_output.estimated_velocity_mps;
    shadow.estimated_disturbance_mps2 =
        g_hball_m55_output.estimated_disturbance_mps2;
    (void)hball_m55_publish_control_shadow(&shadow);
}

static void hball_m55_worker(void *parameter)
{
    TickType_t release_tick = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(HBALL_M55_PERIOD_MS);
    uint32_t previous_step_ms = hball_m55_now_ms();
    uint32_t last_ui_ms = previous_step_ms;

    (void)parameter;
    for (;;)
    {
        hball_sensor_snapshot_t snapshot;
        hball_m55_ui_snapshot_t ui;
        uint32_t now_ms = hball_m55_now_ms();

        if ((uint32_t)(now_ms - previous_step_ms) > (2U * HBALL_M55_PERIOD_MS))
        {
            g_hball_m55_deadline_misses++;
        }
        previous_step_ms = now_ms;
        if (hball_m55_read_sensor_snapshot(&snapshot)
            && (snapshot.sequence != g_hball_m55_snapshot.sequence))
        {
            g_hball_m55_snapshot = snapshot;
            g_hball_m55_input_updates++;
        }
        else
        {
            hball_m55_age_snapshot();
        }

        hball_control_pipeline_step(
            &g_hball_m55_pipeline,
            &g_hball_m55_snapshot,
            0.005F,
            0.0F,
            &g_hball_m55_output
        );
        g_hball_m55_steps++;
        now_ms = hball_m55_now_ms();
        hball_m55_publish_shadow(now_ms);

        if ((uint32_t)(now_ms - last_ui_ms) >= HBALL_M55_UI_PERIOD_MS)
        {
            last_ui_ms = now_ms;
            hball_m55_make_ui_snapshot(&ui);
            taskENTER_CRITICAL();
            g_hball_m55_ui_snapshot = ui;
            taskEXIT_CRITICAL();
            hball_m55_platform_ui_10hz(&ui);
        }
        vTaskDelayUntil(&release_tick, period_ticks);
    }
}

BaseType_t hball_m55_freertos_start(void)
{
    BaseType_t result;

    if (g_hball_m55_task != NULL)
    {
        return pdPASS;
    }
    hball_control_pipeline_init(&g_hball_m55_pipeline, 0.0F);
    memset(&g_hball_m55_snapshot, 0, sizeof(g_hball_m55_snapshot));
    memset(&g_hball_m55_output, 0, sizeof(g_hball_m55_output));
    memset(&g_hball_m55_ui_snapshot, 0, sizeof(g_hball_m55_ui_snapshot));
    g_hball_m55_snapshot.vision_receive_age_ms = UINT32_MAX;
    g_hball_m55_snapshot.imu_age_ms = UINT32_MAX;
    g_hball_m55_snapshot.wheel_age_ms = UINT32_MAX;
    g_hball_m55_snapshot.motor_age_ms = UINT32_MAX;
    g_hball_m55_snapshot.heartbeat_age_ms = UINT32_MAX;

    result = xTaskCreate(
        hball_m55_worker,
        "hball_lqg",
        HBALL_M55_TASK_STACK_WORDS,
        NULL,
        HBALL_M55_TASK_PRIORITY,
        &g_hball_m55_task
    );
    if (result != pdPASS)
    {
        g_hball_m55_task = NULL;
    }
    return result;
}
