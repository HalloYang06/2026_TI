#include "hball_control_guard.h"
#include "hball_dualcore_platform.h"
#include "hball_m33_inputs.h"
#include "hball_diagnostics_config.h"
#include "hball_usb_telemetry.h"

#include <finsh.h>
#include <rtthread.h>

#include <string.h>

#define HBALL_M33_GUARD_PERIOD_MS 1U
#define HBALL_M33_GUARD_LOG_PERIOD_MS 1000U
#define HBALL_M33_TELEMETRY_PERIOD_MS 20U

_Static_assert(
    HBALL_CONTROL_GUARD_ACTUATOR_TX_ENABLED == 0U,
    "H-ball M33 actuator transmission must remain disabled"
);

static hball_control_guard_t g_hball_m33_guard;
static hball_control_decision_t g_hball_m33_decision;
static hball_control_shadow_t g_hball_m33_shadow;
static rt_thread_t g_hball_m33_guard_worker = RT_NULL;
static rt_uint32_t g_hball_m33_ipc_read_total = 0U;
static rt_uint32_t g_hball_m33_ipc_busy_total = 0U;
static rt_uint32_t g_hball_m33_ipc_failure_total = 0U;
static rt_uint32_t g_hball_m33_snapshot_failure_total = 0U;
static rt_uint32_t g_hball_m33_stale_event_total = 0U;
static hball_ipc_result_t g_hball_m33_last_ipc_result = HBALL_IPC_HEADER;
static rt_bool_t g_hball_m33_fresh_previous = RT_FALSE;
static rt_uint32_t g_hball_m33_telemetry_sequence = 0U;
static rt_uint32_t g_hball_m33_last_telemetry_ms = 0U;

static void hball_m33_submit_telemetry(
    const hball_sensor_snapshot_t *sensor, rt_uint32_t now_ms
)
{
    hball_log_record_t record;

    if ((rt_uint32_t)(now_ms - g_hball_m33_last_telemetry_ms)
        < HBALL_M33_TELEMETRY_PERIOD_MS)
    {
        return;
    }
    g_hball_m33_last_telemetry_ms = now_ms;
    rt_memset(&record, 0, sizeof(record));
    record.sequence = g_hball_m33_telemetry_sequence++;
    record.produced_time_ms = now_ms;
    record.sensor_sequence = sensor->sequence;
    record.controller_steps = g_hball_m33_shadow.controller_steps;
    record.vision_sequence = sensor->vision_sequence;
    record.sensor_valid_flags = sensor->valid_flags;
    record.control_mode = g_hball_m33_shadow.mode;
    record.guard_reason = (rt_uint16_t)g_hball_m33_decision.reason;
    record.status_flags =
        g_hball_m33_decision.safety_qualified ? UINT32_C(1) : 0U;
    record.vision_capture_time_us = sensor->vision_capture_time_us;
    record.vision_receive_time_ms = sensor->vision_receive_time_ms;
    record.vision_processing_time_us = sensor->vision_processing_time_us;
    record.imu_epoch = sensor->imu_epoch;
    record.imu_sample_mask = sensor->imu_sample_mask;
    record.imu_source_time_ms = sensor->imu_source_time_ms;
    record.accel_receive_time_ms = sensor->accel_receive_time_ms;
    record.gyro_receive_time_ms = sensor->gyro_receive_time_ms;
    record.attitude_receive_time_ms = sensor->attitude_receive_time_ms;
    record.imu_sync_receive_time_ms = sensor->imu_sync_receive_time_ms;
    record.wheel_sequence = sensor->wheel_sequence;
    record.accel_sequence = sensor->accel_sequence;
    record.gyro_sequence = sensor->gyro_sequence;
    record.attitude_sequence = sensor->attitude_sequence;
    record.wheel_receive_time_ms = sensor->wheel_receive_time_ms;
    record.motor_receive_time_ms = sensor->motor_receive_time_ms;
    record.msp_status_flags = sensor->msp_status_flags;
    record.vision_flags = sensor->vision_flags;
    record.motor_fault_summary = sensor->motor_fault_summary;
    record.motor_mode_state = sensor->motor_mode_state;
    record.motor_run_mode = sensor->motor_run_mode;
    record.vision_age_ms = sensor->vision_receive_age_ms;
    record.imu_age_ms = sensor->imu_age_ms;
    record.wheel_age_ms = sensor->wheel_age_ms;
    record.motor_age_ms = sensor->motor_age_ms;
    record.heartbeat_age_ms = sensor->heartbeat_age_ms;
    record.ball_position_m = sensor->ball_position_m;
    record.vision_confidence = sensor->vision_confidence;
    record.estimated_position_m = g_hball_m33_shadow.estimated_position_m;
    record.estimated_velocity_mps = g_hball_m33_shadow.estimated_velocity_mps;
    record.estimated_disturbance_mps2 =
        g_hball_m33_shadow.estimated_disturbance_mps2;
    record.pipe_target_rad = g_hball_m33_decision.shadow_target_rad;
    record.motor_angle_rad = sensor->motor_angle_rad;
    record.motor_velocity_rad_s = sensor->motor_velocity_rad_s;
    record.motor_torque_nm = sensor->motor_torque_nm;
    record.motor_temperature_c = sensor->motor_temperature_c;
    record.motor_filtered_iq_a = sensor->motor_filtered_iq_a;
    record.motor_vbus_v = sensor->motor_vbus_v;
    memcpy(record.accel_mps2, sensor->accel_mps2,
        sizeof(record.accel_mps2));
    memcpy(record.gyro_rad_s, sensor->gyro_rad_s,
        sizeof(record.gyro_rad_s));
    memcpy(record.attitude_rad, sensor->attitude_rad,
        sizeof(record.attitude_rad));
    record.wheel_left_mps = sensor->wheel_left_mps;
    record.wheel_right_mps = sensor->wheel_right_mps;
    record.body_speed_mps = sensor->body_speed_mps;
    record.controller_command_rad = g_hball_m33_decision.shadow_target_rad;
    (void)hball_usb_telemetry_submit(&record);
}

static void hball_m33_guard_print_status(void)
{
    const rt_uint32_t now_ms = (rt_uint32_t)rt_tick_get_millisecond();
    const rt_bool_t fresh = hball_control_guard_is_fresh(
        &g_hball_m33_guard, now_ms
    ) ? RT_TRUE : RT_FALSE;

    rt_kprintf(
        "[hball-guard] rate_hz=1000 ipc=%lu/%lu/%lu/%d snap_fail=%lu qualified=%lu rejected=%lu duplicate=%lu stale=%lu fresh=%d step=%lu source=%lu reason=%u target_urad=%ld ACTUATOR_TX=0\n",
        (unsigned long)g_hball_m33_ipc_read_total,
        (unsigned long)g_hball_m33_ipc_busy_total,
        (unsigned long)g_hball_m33_ipc_failure_total,
        (int)g_hball_m33_last_ipc_result,
        (unsigned long)g_hball_m33_snapshot_failure_total,
        (unsigned long)g_hball_m33_guard.qualified_total,
        (unsigned long)g_hball_m33_guard.rejected_total,
        (unsigned long)g_hball_m33_guard.duplicate_total,
        (unsigned long)g_hball_m33_stale_event_total,
        (int)fresh,
        (unsigned long)g_hball_m33_shadow.controller_steps,
        (unsigned long)g_hball_m33_shadow.source_sensor_sequence,
        (unsigned)g_hball_m33_guard.last_reason,
        (long)(g_hball_m33_decision.shadow_target_rad * 1000000.0F)
    );
}

static void hball_m33_guard_step(rt_uint32_t now_ms)
{
    hball_control_shadow_t shadow;

    g_hball_m33_last_ipc_result = hball_ipc_control_read(
        &hball_ipc_platform_region()->control,
        &shadow,
        hball_ipc_platform_cache_ops()
    );
    if (g_hball_m33_last_ipc_result == HBALL_IPC_OK)
    {
        hball_sensor_snapshot_t sensor;

        g_hball_m33_ipc_read_total++;
        g_hball_m33_shadow = shadow;
        if (hball_m33_inputs_get_snapshot(&sensor))
        {
            (void)hball_control_guard_observe(
                &g_hball_m33_guard,
                &shadow,
                &sensor,
                now_ms,
                &g_hball_m33_decision
            );
            hball_m33_submit_telemetry(&sensor, now_ms);
        }
        else
        {
            g_hball_m33_snapshot_failure_total++;
        }
    }
    else if (g_hball_m33_last_ipc_result == HBALL_IPC_BUSY)
    {
        g_hball_m33_ipc_busy_total++;
    }
    else
    {
        g_hball_m33_ipc_failure_total++;
    }
    {
        const rt_bool_t fresh = hball_control_guard_is_fresh(
            &g_hball_m33_guard, now_ms
        ) ? RT_TRUE : RT_FALSE;

        if (g_hball_m33_fresh_previous && !fresh)
        {
            g_hball_m33_stale_event_total++;
        }
        g_hball_m33_fresh_previous = fresh;
    }
}

static void hball_m33_guard_worker_entry(void *parameter)
{
    rt_tick_t release_tick = rt_tick_get();
    const rt_tick_t period_ticks =
        rt_tick_from_millisecond(HBALL_M33_GUARD_PERIOD_MS);
#if HBALL_PERIODIC_DIAGNOSTICS
    rt_uint32_t last_log_ms = (rt_uint32_t)rt_tick_get_millisecond();
#endif

    RT_UNUSED(parameter);
    while (1)
    {
        const rt_uint32_t now_ms =
            (rt_uint32_t)rt_tick_get_millisecond();

        hball_m33_guard_step(now_ms);
#if HBALL_PERIODIC_DIAGNOSTICS
        if ((rt_uint32_t)(now_ms - last_log_ms)
            >= HBALL_M33_GUARD_LOG_PERIOD_MS)
        {
            last_log_ms = now_ms;
            hball_m33_guard_print_status();
        }
#endif
        (void)rt_thread_delay_until(&release_tick, period_ticks);
    }
}

static void hball_guard_status(void)
{
    hball_m33_guard_print_status();
}
MSH_CMD_EXPORT(hball_guard_status, show M33 shadow-only safety diagnostics);

static int hball_m33_control_guard_init(void)
{
    hball_control_guard_init(&g_hball_m33_guard);
    rt_memset(&g_hball_m33_decision, 0, sizeof(g_hball_m33_decision));
    rt_memset(&g_hball_m33_shadow, 0, sizeof(g_hball_m33_shadow));
    g_hball_m33_guard_worker = rt_thread_create(
        "hball_safe",
        hball_m33_guard_worker_entry,
        RT_NULL,
        3072U,
        13U,
        5U
    );
    if (g_hball_m33_guard_worker == RT_NULL)
    {
        return -RT_ERROR;
    }
    rt_thread_startup(g_hball_m33_guard_worker);
    rt_kprintf(
        "[hball-guard] M33 1 kHz shadow observer; hard-disabled ACTUATOR_TX=0\n"
    );
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(hball_m33_control_guard_init);
