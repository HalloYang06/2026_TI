#include "hball_control_pipeline.h"
#include "hball_m55_ipc.h"
#include "hball_m55_ui.h"

#include <finsh.h>
#include <rtthread.h>

#define HBALL_M55_VERSION "0.2.0-multirate-shadow"
#define HBALL_M55_PERIOD_MS 5U
#define HBALL_M55_LOG_PERIOD_MS 1000U

static hball_control_pipeline_t g_hball_m55_pipeline;
static hball_control_output_t g_hball_m55_output;
static hball_sensor_snapshot_t g_hball_m55_snapshot;
static rt_thread_t g_hball_m55_worker = RT_NULL;
static rt_uint32_t g_hball_m55_steps = 0U;
static rt_uint32_t g_hball_m55_deadline_misses = 0U;
static rt_uint32_t g_hball_m55_input_updates = 0U;

extern int lvgl_thread_init(void);

void hball_m55_get_ui_snapshot(hball_m55_ui_snapshot_t *snapshot)
{
    if (snapshot == RT_NULL)
    {
        return;
    }

    rt_memset(snapshot, 0, sizeof(*snapshot));
    snapshot->controller_steps = g_hball_m55_steps;
    snapshot->deadline_misses = g_hball_m55_deadline_misses;
    snapshot->vision_age_ms = g_hball_m55_snapshot.vision_receive_age_ms;
    snapshot->ball_position_m = g_hball_m55_output.estimated_position_m;
    snapshot->ball_velocity_mps = g_hball_m55_output.estimated_velocity_mps;
    snapshot->longitudinal_accel_mps2 =
        g_hball_m55_snapshot.longitudinal_accel_mps2;
    snapshot->yaw_rate_rad_s = g_hball_m55_snapshot.yaw_rate_rad_s;
    snapshot->motor_angle_rad = g_hball_m55_snapshot.motor_angle_rad;
    snapshot->motor_velocity_rad_s = g_hball_m55_snapshot.motor_velocity_rad_s;
    snapshot->motor_temperature_c = g_hball_m55_snapshot.motor_temperature_c;
    snapshot->motor_filtered_iq_a =
        g_hball_m55_snapshot.motor_filtered_iq_a;
    snapshot->motor_vbus_v = g_hball_m55_snapshot.motor_vbus_v;
    snapshot->motor_mode_state = g_hball_m55_snapshot.motor_mode_state;
    snapshot->motor_run_mode = g_hball_m55_snapshot.motor_run_mode;
    snapshot->motor_fault_summary = g_hball_m55_snapshot.motor_fault_summary;
    snapshot->lqg_target_rad = g_hball_m55_output.shadow_command_rad;
    if ((g_hball_m55_snapshot.valid_flags & HBALL_SENSOR_VALID_IMU) != 0U)
    {
        snapshot->valid_flags |= HBALL_UI_VALID_IMU;
    }
    if ((g_hball_m55_snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) != 0U)
    {
        snapshot->valid_flags |= HBALL_UI_VALID_VISION;
    }
    if ((g_hball_m55_snapshot.valid_flags & HBALL_SENSOR_VALID_MOTOR) != 0U)
    {
        snapshot->valid_flags |= HBALL_UI_VALID_MOTOR;
    }
    if ((g_hball_m55_snapshot.valid_flags
            & HBALL_SENSOR_VALID_MOTOR_PARAMETERS) != 0U)
    {
        snapshot->valid_flags |= HBALL_UI_VALID_MOTOR_PARAMETERS;
    }
}

static void hball_m55_print_status(void)
{
    hball_m55_ipc_diag_t ipc;

    hball_m55_ipc_get_diag(&ipc);
    rt_kprintf(
        "[hball-m55] version=%s rate_hz=200 steps=%lu input=%lu misses=%lu ipc_rx=%lu/%lu/%lu ipc_tx=%lu/%lu mode=%u eligible=%d cmd_urad=%ld ball_um=%ld vel_um_s=%ld ACTUATOR_TX=0\n",
        HBALL_M55_VERSION,
        (unsigned long)g_hball_m55_steps,
        (unsigned long)g_hball_m55_input_updates,
        (unsigned long)g_hball_m55_deadline_misses,
        (unsigned long)ipc.sensor_read_total,
        (unsigned long)ipc.sensor_busy_total,
        (unsigned long)ipc.sensor_failure_total,
        (unsigned long)ipc.control_publish_total,
        (unsigned long)ipc.control_failure_total,
        (unsigned)g_hball_m55_output.mode,
        (int)g_hball_m55_output.safety_eligible,
        (long)(g_hball_m55_output.shadow_command_rad * 1000000.0F),
        (long)(g_hball_m55_output.estimated_position_m * 1000000.0F),
        (long)(g_hball_m55_output.estimated_velocity_mps * 1000000.0F)
    );
}

static rt_uint32_t hball_m55_add_age(rt_uint32_t age_ms)
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

static void hball_m55_worker_entry(void *parameter)
{
    rt_uint32_t last_log_ms = (rt_uint32_t)rt_tick_get_millisecond();
    rt_uint32_t previous_step_ms = last_log_ms;
    rt_tick_t release_tick = rt_tick_get();
    const rt_tick_t period_ticks =
        rt_tick_from_millisecond(HBALL_M55_PERIOD_MS);

    RT_UNUSED(parameter);
    while (1)
    {
        rt_uint32_t now_ms = (rt_uint32_t)rt_tick_get_millisecond();

        if ((rt_uint32_t)(now_ms - previous_step_ms) > (2U * HBALL_M55_PERIOD_MS))
        {
            g_hball_m55_deadline_misses++;
        }
        previous_step_ms = now_ms;
        {
            hball_sensor_snapshot_t snapshot;

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
        }
        hball_control_pipeline_step(
            &g_hball_m55_pipeline,
            &g_hball_m55_snapshot,
            0.005F,
            0.0F,
            &g_hball_m55_output
        );
        g_hball_m55_steps++;
        {
            hball_control_shadow_t shadow;

            rt_memset(&shadow, 0, sizeof(shadow));
            shadow.source_sensor_sequence = g_hball_m55_snapshot.sequence;
            shadow.controller_steps = g_hball_m55_steps;
            shadow.deadline_misses = g_hball_m55_deadline_misses;
            shadow.produced_time_ms =
                (rt_uint32_t)rt_tick_get_millisecond();
            shadow.mode = (uint16_t)g_hball_m55_output.mode;
            shadow.flags = HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY;
            if (g_hball_m55_output.safety_eligible)
            {
                shadow.flags |= HBALL_IPC_CONTROL_FLAG_SAFETY_ELIGIBLE;
            }
            shadow.target_angle_rad = g_hball_m55_output.shadow_command_rad;
            shadow.estimated_position_m =
                g_hball_m55_output.estimated_position_m;
            shadow.estimated_velocity_mps =
                g_hball_m55_output.estimated_velocity_mps;
            shadow.estimated_disturbance_mps2 =
                g_hball_m55_output.estimated_disturbance_mps2;
            (void)hball_m55_publish_control_shadow(&shadow);
        }

        now_ms = (rt_uint32_t)rt_tick_get_millisecond();
        if ((rt_uint32_t)(now_ms - last_log_ms) >= HBALL_M55_LOG_PERIOD_MS)
        {
            last_log_ms = now_ms;
            hball_m55_print_status();
        }
        (void)rt_thread_delay_until(&release_tick, period_ticks);
    }
}

static void hball_m55_status(void)
{
    hball_m55_print_status();
}
MSH_CMD_EXPORT(hball_m55_status, show M55 200 Hz LQG shadow diagnostics);

static int hball_m55_shadow_start(void)
{
    int lvgl_result;

    hball_control_pipeline_init(&g_hball_m55_pipeline, 0.0F);
    rt_memset(&g_hball_m55_snapshot, 0, sizeof(g_hball_m55_snapshot));
    rt_memset(&g_hball_m55_output, 0, sizeof(g_hball_m55_output));
    g_hball_m55_snapshot.vision_receive_age_ms = UINT32_MAX;
    g_hball_m55_snapshot.imu_age_ms = UINT32_MAX;
    g_hball_m55_snapshot.motor_age_ms = UINT32_MAX;
    g_hball_m55_snapshot.heartbeat_age_ms = UINT32_MAX;
    rt_kprintf(
        "[hball-m55] SAFETY multirate LQG shadow only; no CAN or actuator output\n"
    );

    g_hball_m55_worker = rt_thread_create(
        "hball_lqg",
        hball_m55_worker_entry,
        RT_NULL,
        4096U,
        12U,
        10U
    );
    if (g_hball_m55_worker == RT_NULL)
    {
        return -RT_ERROR;
    }
    rt_thread_startup(g_hball_m55_worker);
    lvgl_result = lvgl_thread_init();
    rt_kprintf("[hball-m55] LVGL H-ball page init=%d\n", lvgl_result);
    return RT_EOK;
}
INIT_APP_EXPORT(hball_m55_shadow_start);
