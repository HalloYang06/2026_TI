#include "hball_can.h"
#include "hball_rate_meter.h"
#include "hball_diagnostics_config.h"
#include "hball_m33_q456.h"
#include "hball_mission_arbiter.h"
#include "hball_mission_can.h"
#include "hball_rs00_control.h"
#include "hball_usb_telemetry.h"
#if HBALL_INTEGRATED_SHADOW
#include "hball_control_pipeline.h"
#include "hball_deployment_controller.h"
#include "hball_deployment_config.h"
#include "hball_m33_inputs.h"
#include "hball_runtime_tuning.h"
#endif

#include "drv_can.h"
#include <finsh.h>
#include <math.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <stdlib.h>
#include <string.h>

#ifndef HBALL_BENCH_AUTO_PROBE5
#define HBALL_BENCH_AUTO_PROBE5 0
#endif

#ifndef HBALL_RS00_READBACK_TX_ENABLED
#define HBALL_RS00_READBACK_TX_ENABLED 0
#endif

#ifndef HBALL_RS00_MOTION_TX_ENABLED
#define HBALL_RS00_MOTION_TX_ENABLED 0
#endif

#if HBALL_INTEGRATED_SHADOW
#define HBALL_BENCH_VERSION "0.7.0-q456-oosm-runtime"
#else
#define HBALL_BENCH_VERSION "0.3.0-m33-can-readback"
#endif
#define HBALL_BENCH_PERIOD_MS 1U
#define HBALL_BENCH_LOG_PERIOD_MS 1000U
#define HBALL_BENCH_AUTO_PROBE_DELAY_MS 1000U
#define HBALL_BENCH_RX_BUDGET 32U
#define HBALL_MISSION_STATUS_PERIOD_MS 50U
#define HBALL_MISSION_READY_SENSOR_FRESH_MS 100U
#define HBALL_RS00_READBACK_PERIOD_MS 20U
#define HBALL_RS00_READBACK_TIMEOUT_MS 10U
#define HBALL_RS00_MOTION_PARAMETER_FRESH_MS 250U
#define HBALL_RS00_MOTION_PREPARE_TIMEOUT_MS 250U
#define HBALL_RS00_MOTION_RETURN_TIMEOUT_MS 500U
#define HBALL_RS00_MOTION_VBUS_MIN_V 18.0F
#define HBALL_RS00_MOTION_VBUS_MAX_V 26.0F
#define HBALL_RS00_MOTION_STATIONARY_RAD_S 0.2F
#define HBALL_RS00_MOTION_RETURN_TOLERANCE_RAD 0.002F
#define HBALL_RS00_MOTION_RETURN_VELOCITY_RAD_S 0.05F
#define HBALL_RS00_MOTION_RETURN_SETTLE_MS 50U
#define HBALL_RS00_MOTION_READBACK_PERIOD_MS 5U
#define HBALL_RS00_MOTION_READBACK_FRESH_MS 100U
#define HBALL_BALL_SENSOR_GRACE_MS 500U
#define HBALL_RS00_STEP_TRACE_CAPACITY 120U
#define HBALL_RS00_STEP_TRACE_DURATION_MS 1000U
#define HBALL_RS00_PARAMETER_SLOT_MECH_POSITION 1U
#define HBALL_RS00_PARAMETER_SLOT_MECH_VELOCITY 3U
#define HBALL_RS00_CONFIRM_TOKEN "CONFIRM_NO_LOAD"
#define HBALL_BALL_COMMISSION_LEVEL_RAD 1.7205F
#define HBALL_BALL_COMMISSION_PIPE_LIMIT_RAD 0.052359878F
#define HBALL_BALL_COMMISSION_RECOVERY_LIMIT_RAD 0.052359878F
#define HBALL_BALL_FORMAL_PIPE_LIMIT_RAD 0.034906585F
#define HBALL_BALL_FORMAL_RECOVERY_LIMIT_RAD 0.043633231F
#define HBALL_BALL_Q45_STARTUP_PIPE_LIMIT_RAD 0.041887902F
#define HBALL_BALL_Q45_STARTUP_LIMIT_MS 1000U
#define HBALL_BALL_SETTLE_PIPE_LIMIT_RAD 0.017453293F
#define HBALL_BALL_SETTLE_CAPTURE_LIMIT_RAD 0.034906585F
#define HBALL_BALL_SETTLE_RECOVERY_LIMIT_RAD 0.052359879F
#define HBALL_BALL_Q3_PRELEVEL_PIPE_RATE_LIMIT_RAD_S 0.50F
#define HBALL_BALL_Q3_TARGET_RATE_MPS 0.20F
#define HBALL_BALL_VISION_HOLD_MS 100U
#define HBALL_BALL_PID_KP 0.70F
#define HBALL_BALL_PID_KI 0.15F
#define HBALL_BALL_PID_KD 0.15F
#define HBALL_BALL_PID_BOOST_ENTER_MPS 0.003F
#define HBALL_BALL_PID_BOOST_EXIT_MPS 0.015F
#define HBALL_BALL_LQI_KP 1.576194F
#define HBALL_BALL_LQI_KI 1.000000F
#define HBALL_BALL_LQI_KD 0.713641F
#define HBALL_BALL_PID_INTEGRAL_LIMIT 0.050F
#define HBALL_BALL_COMMISSION_POSITION_LIMIT_M 0.120F
#define HBALL_BALL_Q3_START_WINDOW_M 0.010F
#define HBALL_BALL_Q3_LEVEL_SETTLE_MS 100U
#define HBALL_BALL_CONTROL_PERIOD_MS 2U
#define HBALL_BALL_CONTROL_DT_S 0.002F
#define HBALL_BALL_COMMISSION_TX_PERIOD_MS 2U
#define HBALL_BALL_COMMISSION_TIMEOUT_MS 15000U
/* Formal Q4-Q6 hold ends only on a new mission command or a hard fault. */
#define HBALL_BALL_HOLD_TIMEOUT_MS 0U

#ifndef BSP_CANFD0_RX_FIFO0_ELEMENTS
#error "H-ball CAN build must define the RX FIFO depth"
#endif

_Static_assert(
    BSP_CANFD0_RX_FIFO0_ELEMENTS >= 64U,
    "H-ball CAN RX FIFO must absorb bounded diagnostic UART stalls"
);

static hball_motor_monitor_t g_hball_motor;
static hball_msp_monitor_t g_hball_msp;
static hball_can_frame_t g_hball_last_rx;
static rt_thread_t g_hball_worker = RT_NULL;
static rt_bool_t g_hball_can_ready = RT_FALSE;
static rt_bool_t g_hball_last_rx_valid = RT_FALSE;
static rt_uint32_t g_hball_raw_rx_total = 0U;
static rt_uint32_t g_hball_tx_total = 0U;
static rt_uint32_t g_hball_tx_success = 0U;
static rt_uint32_t g_hball_tx_failure = 0U;
static hball_rate_meter_t g_hball_can_rx_rate;
static hball_mission_arbiter_t g_hball_mission_arbiter;
static hball_mission_chassis_status_t g_hball_mission_chassis;
static hball_m33_q456_t g_hball_q456_runtime;
static rt_uint32_t g_hball_mission_intent_rx = 0U;
static rt_uint32_t g_hball_mission_intent_invalid = 0U;
static rt_uint8_t g_hball_mission_last_invalid_intent[8];
static rt_uint32_t g_hball_mission_chassis_rx = 0U;
static rt_uint32_t g_hball_mission_chassis_invalid = 0U;
static rt_uint32_t g_hball_mission_status_tx = 0U;
static rt_uint32_t g_hball_mission_status_tx_failure = 0U;
static rt_uint32_t g_hball_mission_last_status_ms = 0U;
#if HBALL_RS00_MOTION_TX_ENABLED
typedef enum
{
    HBALL_MOTION_REQUEST_NONE = 0,
    HBALL_MOTION_REQUEST_PROBE,
    HBALL_MOTION_REQUEST_PREPARE,
    HBALL_MOTION_REQUEST_ARM,
    HBALL_MOTION_REQUEST_STEP,
    HBALL_MOTION_REQUEST_RETURN,
    HBALL_MOTION_REQUEST_STOP,
} hball_motion_request_t;

static hball_rs00_bench_t g_hball_motion;
static rt_mutex_t g_hball_motion_mutex = RT_NULL;
static hball_motion_request_t g_hball_motion_request =
    HBALL_MOTION_REQUEST_NONE;
static float g_hball_motion_request_step_rad = 0.0F;
static rt_uint8_t g_hball_motion_prepare_stage = 0U;
static rt_uint32_t g_hball_motion_next_tx_ms = 0U;
static rt_uint32_t g_hball_motion_mode_read_rx_baseline = 0U;
static rt_uint32_t g_hball_motion_mode_read_ms = 0U;
static rt_uint32_t g_hball_motion_tx_total = 0U;
static rt_uint32_t g_hball_motion_reject_total = 0U;
static rt_uint32_t g_hball_motion_auto_stop_total = 0U;
static rt_uint32_t g_hball_motion_return_settle_since_ms = 0U;
static rt_uint32_t g_hball_motion_readback_last_ms = 0U;
static rt_uint32_t g_hball_motion_readback_tx_total = 0U;
static rt_uint8_t g_hball_motion_readback_next = 0U;
static void hball_motion_begin_prepare(rt_uint32_t now_ms);
static void hball_motion_arm(rt_uint32_t now_ms);
typedef struct
{
    rt_uint32_t elapsed_ms;
    float target_rad;
    float position_rad;
    float velocity_rad_s;
    rt_uint8_t fault;
} hball_step_trace_sample_t;
static hball_step_trace_sample_t
    g_hball_step_trace[HBALL_RS00_STEP_TRACE_CAPACITY];
static rt_uint32_t g_hball_step_trace_start_ms = 0U;
static rt_uint32_t g_hball_step_trace_last_position_ms = 0U;
static rt_uint16_t g_hball_step_trace_count = 0U;
static rt_bool_t g_hball_step_trace_active = RT_FALSE;
#if HBALL_INTEGRATED_SHADOW
static hball_control_pipeline_t g_hball_ball_pipeline;
static hball_control_output_t g_hball_ball_output;
static rt_bool_t g_hball_ball_active = RT_FALSE;
static float g_hball_ball_target_m = 0.0F;
static float g_hball_ball_level_rad = HBALL_BALL_COMMISSION_LEVEL_RAD;
static rt_uint32_t g_hball_ball_start_ms = 0U;
static rt_uint32_t g_hball_ball_last_step_ms = 0U;
static rt_uint32_t g_hball_ball_sensor_invalid_since_ms = 0U;
static rt_bool_t g_hball_ball_vision_lost = RT_FALSE;
static rt_uint32_t g_hball_ball_last_tx_ms = 0U;
static rt_uint32_t g_hball_ball_tx_total = 0U;
static rt_uint8_t g_hball_ball_phase = 0U;
static rt_uint32_t g_hball_ball_settle_since_ms = 0U;
static rt_bool_t g_hball_ball_q3_passed = RT_FALSE;
static float g_hball_ball_position_integral = 0.0F;
static float g_hball_ball_previous_error_m = 0.0F;
static float g_hball_ball_pid_kp = HBALL_BALL_PID_KP;
static float g_hball_ball_pid_ki = HBALL_BALL_PID_KI;
static float g_hball_ball_pid_kd = HBALL_BALL_PID_KD;
static float g_hball_ball_pid_static_boost_rad = 0.0F;
static rt_bool_t g_hball_ball_pid_static_boost_active = RT_FALSE;
static float g_hball_ball_q3_pipe_command_rad = 0.0F;
static rt_uint8_t g_hball_ball_mode = 0U;
static rt_bool_t g_hball_ball_use_lqi = RT_FALSE;
static float g_hball_ball_max_abs_error_m = 0.0F;
static rt_uint32_t g_hball_ball_error_violation_total = 0U;
static rt_uint32_t g_hball_mission_action_last_ms = 0U;
static rt_uint32_t g_hball_ball_log_sequence = 0U;
static rt_uint32_t g_hball_ball_last_log_ms = 0U;
static rt_uint16_t g_hball_ball_control_epoch = 0U;
static rt_uint8_t g_hball_ball_control_mission = 0U;
static rt_bool_t g_hball_ball_settle_hold = RT_FALSE;
static rt_bool_t g_hball_ball_q3_leveling = RT_FALSE;
static rt_bool_t g_hball_ball_q3_leveling_first_target = RT_FALSE;
static float g_hball_formal_pipe_limit_rad =
    HBALL_BALL_FORMAL_PIPE_LIMIT_RAD;
static float g_hball_formal_recovery_limit_rad =
    HBALL_BALL_FORMAL_RECOVERY_LIMIT_RAD;
static float g_hball_settle_pipe_limit_rad =
    HBALL_BALL_SETTLE_PIPE_LIMIT_RAD;
static float g_hball_settle_capture_limit_rad =
    HBALL_BALL_SETTLE_CAPTURE_LIMIT_RAD;
static float g_hball_settle_recovery_limit_rad =
    HBALL_BALL_SETTLE_RECOVERY_LIMIT_RAD;
static float g_hball_lqi_position_gain = 2.64956F;
static float g_hball_lqi_velocity_gain = 1.050000F;
static float g_hball_lqi_integral_gain = 0.787185F;

bool hball_runtime_tuning_set(const char *name, float value)
{
    float radians;

    if ((name == RT_NULL) || !isfinite(value) || g_hball_ball_active)
    {
        return false;
    }
    if ((strcmp(name, "kp") == 0) || (strcmp(name, "kv") == 0)
        || (strcmp(name, "ki") == 0))
    {
        float kp = g_hball_lqi_position_gain;
        float kv = g_hball_lqi_velocity_gain;
        float ki = g_hball_lqi_integral_gain;

        if (strcmp(name, "kp") == 0) kp = value;
        if (strcmp(name, "kv") == 0) kv = value;
        if (strcmp(name, "ki") == 0) ki = value;
        if (!hball_deployment_controller_set_gains(kp, kv, ki))
        {
            return false;
        }
        g_hball_lqi_position_gain = kp;
        g_hball_lqi_velocity_gain = kv;
        g_hball_lqi_integral_gain = ki;
        return true;
    }
    if ((value < 0.25F) || (value > 6.0F))
    {
        return false;
    }
    radians = value * 0.01745329252F;
    if (strcmp(name, "run_deg") == 0)
        g_hball_formal_pipe_limit_rad = radians;
    else if (strcmp(name, "run_recovery_deg") == 0)
        g_hball_formal_recovery_limit_rad = radians;
    else if (strcmp(name, "settle_deg") == 0)
        g_hball_settle_pipe_limit_rad = radians;
    else if (strcmp(name, "settle_capture_deg") == 0)
        g_hball_settle_capture_limit_rad = radians;
    else if (strcmp(name, "settle_recovery_deg") == 0)
        g_hball_settle_recovery_limit_rad = radians;
    else
        return false;
    return true;
}
static int hball_q3_start_common(void);
static int hball_hold_start_common(
    rt_uint8_t mode, float target_m, const char *name
);
static void hball_ball_submit_actual_log(
    const hball_sensor_snapshot_t *snapshot, rt_uint32_t now_ms
);
#endif
#endif
#if HBALL_RS00_READBACK_TX_ENABLED
static const uint16_t g_hball_rs00_readback_indexes[] = {
    HBALL_RS00_INDEX_RUN_MODE,
    HBALL_RS00_INDEX_MECH_POSITION,
    HBALL_RS00_INDEX_FILTERED_IQ,
    HBALL_RS00_INDEX_MECH_VELOCITY,
    HBALL_RS00_INDEX_VBUS,
    HBALL_RS00_INDEX_ROTATION,
};
static rt_uint8_t g_hball_rs00_readback_next = 0U;
static rt_uint32_t g_hball_rs00_readback_last_ms = 0U;
#endif

static rt_uint32_t hball_now_ms(void)
{
    return (rt_uint32_t)rt_tick_get_millisecond();
}

static void hball_copy_from_rt_can(
    hball_can_frame_t *target, const struct rt_can_msg *source
)
{
    rt_uint8_t length = source->len;

    if (length > 8U)
    {
        length = 8U;
    }
    rt_memset(target, 0, sizeof(*target));
    target->id = source->id;
    target->is_extended = (source->ide == RT_CAN_EXTID) ? 1U : 0U;
    target->is_remote = (source->rtr == RT_CAN_RTR) ? 1U : 0U;
    target->dlc = length;
    if (length > 0U)
    {
        rt_memcpy(target->data, source->data, length);
    }
}

static rt_err_t hball_send_can_frame(const hball_can_frame_t *frame)
{
    struct rt_can_msg message;
    rt_err_t result;
    rt_bool_t allowed_extended;
    rt_bool_t allowed_standard;

    if (frame == RT_NULL)
    {
        return -RT_ERROR;
    }
    allowed_extended = frame->is_extended != 0U;
    allowed_standard = (frame->is_extended == 0U)
        && ((frame->id == HBALL_CAN_ID_MISSION_STATUS)
            || (frame->id == HBALL_CAN_ID_MISSION_UI));
    if (!g_hball_can_ready || (frame->is_remote != 0U)
        || (frame->dlc != 8U)
        || (!allowed_extended && !allowed_standard))
    {
        return -RT_ERROR;
    }
    rt_memset(&message, 0, sizeof(message));
    message.id = frame->id;
    message.ide = allowed_extended ? RT_CAN_EXTID : RT_CAN_STDID;
    message.rtr = RT_CAN_DTR;
    message.len = frame->dlc;
    message.hdr_index = -1;
    rt_memcpy(message.data, frame->data, frame->dlc);

    result = ifx_can_direct_send(&message);
    g_hball_tx_total++;
    if (result == RT_EOK)
    {
        g_hball_tx_success++;
    }
    else
    {
        g_hball_tx_failure++;
    }
    return result;
}

static rt_err_t hball_send_mission_frame(
    const hball_mission_can_frame_t *frame
)
{
    hball_can_frame_t can_frame;

    if ((frame == RT_NULL)
        || (frame->is_extended != 0U) || (frame->is_remote != 0U)
        || (frame->dlc != HBALL_MISSION_CAN_DLC)
        || ((frame->id != HBALL_CAN_ID_MISSION_STATUS)
            && (frame->id != HBALL_CAN_ID_MISSION_UI)))
    {
        return -RT_ERROR;
    }
    rt_memset(&can_frame, 0, sizeof(can_frame));
    can_frame.id = frame->id;
    can_frame.dlc = frame->dlc;
    rt_memcpy(can_frame.data, frame->data, frame->dlc);
    return hball_send_can_frame(&can_frame);
}

static void hball_copy_to_mission_frame(
    hball_mission_can_frame_t *target, const hball_can_frame_t *source
)
{
    rt_memset(target, 0, sizeof(*target));
    target->id = source->id;
    target->is_extended = source->is_extended;
    target->is_remote = source->is_remote;
    target->dlc = source->dlc;
    rt_memcpy(target->data, source->data, sizeof(target->data));
}

static void hball_poll_can(void)
{
    struct rt_can_msg message;
    hball_can_frame_t frame;
    rt_uint8_t drained = 0U;

    while ((drained < HBALL_BENCH_RX_BUDGET)
        && (ifx_can_direct_recv(&message) == (rt_ssize_t)sizeof(message)))
    {
        const rt_uint32_t now_ms = hball_now_ms();

        drained++;
        hball_copy_from_rt_can(&frame, &message);
        g_hball_last_rx = frame;
        g_hball_last_rx_valid = RT_TRUE;
        g_hball_raw_rx_total++;
        hball_rate_meter_accept(&g_hball_can_rx_rate, now_ms);
        if (frame.is_extended != 0U)
        {
            const hball_can_event_t event = hball_motor_monitor_accept(
                &g_hball_motor, &frame, now_ms
            );

            if (event == HBALL_CAN_EVENT_PROBE_REPLY)
            {
                rt_kprintf(
                    "[hball-m33] probe5 reply uid=%08lx%08lx rx_total=%lu\n",
                    (unsigned long)(g_hball_motor.unique_id >> 32),
                    (unsigned long)(g_hball_motor.unique_id & 0xFFFFFFFFULL),
                    (unsigned long)g_hball_raw_rx_total
                );
            }
            else if (event == HBALL_CAN_EVENT_FEEDBACK)
            {
#if HBALL_RS00_MOTION_TX_ENABLED
                const hball_rs00_bench_state_t previous_motion_state =
                    g_hball_motion.state;
                (void)hball_rs00_bench_accept_feedback(
                    &g_hball_motion,
                    g_hball_motor.feedback.fault_summary,
                    g_hball_motor.last_feedback_ms,
                    now_ms
                );
                if ((previous_motion_state == HBALL_RS00_BENCH_ARMING)
                    && (g_hball_motion.state == HBALL_RS00_BENCH_ARMED))
                {
                    g_hball_motion.last_manual_command_ms = now_ms;
                    rt_kprintf(
                        "[hball-motion] ARMED by post-enable 0x02 feedback pos_mrad=%ld vel_mrad_s=%ld fault=0x%02x\n",
                        (long)(g_hball_motor.feedback.position_rad * 1000.0F),
                        (long)(g_hball_motor.feedback.velocity_rad_s * 1000.0F),
                        (unsigned int)g_hball_motor.feedback.fault_summary
                    );
                }
#endif
#if HBALL_INTEGRATED_SHADOW
                (void)hball_m33_inputs_publish_motor(
                    &g_hball_motor.feedback, now_ms
                );
#endif
            }
            else if (event == HBALL_CAN_EVENT_PARAMETER)
            {
#if HBALL_RS00_MOTION_TX_ENABLED
                if (g_hball_step_trace_active
                    && (g_hball_motor.parameters.last_update_ms[
                            HBALL_RS00_PARAMETER_SLOT_MECH_POSITION]
                        == now_ms)
                    && (now_ms != g_hball_step_trace_last_position_ms))
                {
                    const rt_uint32_t elapsed_ms =
                        now_ms - g_hball_step_trace_start_ms;

                    if ((elapsed_ms <= HBALL_RS00_STEP_TRACE_DURATION_MS)
                        && (g_hball_step_trace_count
                            < HBALL_RS00_STEP_TRACE_CAPACITY))
                    {
                        hball_step_trace_sample_t *sample =
                            &g_hball_step_trace[g_hball_step_trace_count++];

                        sample->elapsed_ms = elapsed_ms;
                        sample->target_rad =
                            g_hball_motion.target_position_rad;
                        sample->position_rad =
                            g_hball_motor.parameters.mech_position_rad;
                        sample->velocity_rad_s =
                            g_hball_motor.parameters.mech_velocity_rad_s;
                        sample->fault =
                            g_hball_motor.feedback.fault_summary;
                        g_hball_step_trace_last_position_ms = now_ms;
                    }
                    if ((elapsed_ms >= HBALL_RS00_STEP_TRACE_DURATION_MS)
                        || (g_hball_step_trace_count
                            >= HBALL_RS00_STEP_TRACE_CAPACITY))
                    {
                        g_hball_step_trace_active = RT_FALSE;
                    }
                }
#endif
#if HBALL_INTEGRATED_SHADOW
                (void)hball_m33_inputs_publish_motor_parameters(
                    &g_hball_motor.parameters
                );
#endif
            }
        }
        else
        {
            if (frame.id == HBALL_CAN_ID_MISSION_INTENT)
            {
                hball_mission_can_frame_t mission_frame;
                hball_mission_intent_t intent;

                hball_copy_to_mission_frame(&mission_frame, &frame);
                if (hball_mission_decode_intent(&mission_frame, &intent)
                    && hball_mission_arbiter_accept_intent(
                        &g_hball_mission_arbiter, &intent, now_ms))
                {
                    g_hball_mission_intent_rx++;
                }
                else
                {
                    rt_memcpy(
                        g_hball_mission_last_invalid_intent,
                        frame.data,
                        sizeof(g_hball_mission_last_invalid_intent)
                    );
                    g_hball_mission_intent_invalid++;
                }
            }
            else if (frame.id == HBALL_CAN_ID_MISSION_CHASSIS_STATUS)
            {
                hball_mission_can_frame_t mission_frame;

                hball_copy_to_mission_frame(&mission_frame, &frame);
                if (hball_mission_decode_chassis_status(
                        &mission_frame, &g_hball_mission_chassis))
                {
                    g_hball_mission_chassis_rx++;
                }
                else
                {
                    g_hball_mission_chassis_invalid++;
                }
            }
            else
            {
                const hball_msp_event_t event = hball_msp_monitor_accept(
                    &g_hball_msp, &frame, now_ms
                );

#if HBALL_INTEGRATED_SHADOW
                if ((event >= HBALL_MSP_EVENT_HEARTBEAT)
                    && (event <= HBALL_MSP_EVENT_ATTITUDE))
                {
                    (void)hball_m33_inputs_publish_msp(&g_hball_msp);
                }
#else
                RT_UNUSED(event);
#endif
            }
        }
    }
}

static rt_uint16_t hball_mission_ready_mask(rt_uint32_t now_ms)
{
    rt_uint16_t ready = HBALL_MISSION_READY_M33_ALIVE;

    if (hball_msp_monitor_heartbeat_fresh(&g_hball_msp, now_ms, 100U))
    {
        ready |= HBALL_MISSION_READY_MSP_LINK;
        if ((g_hball_msp.status_flags & HBALL_MSP_STATUS_CHASSIS_READY) != 0U)
        {
            ready |= HBALL_MISSION_READY_CHASSIS;
        }
    }
    if (hball_msp_monitor_imu_fresh(
            &g_hball_msp, now_ms, HBALL_MISSION_READY_SENSOR_FRESH_MS)
        && ((g_hball_msp.status_flags & HBALL_MSP_STATUS_IMU_VALID) != 0U))
    {
        ready |= HBALL_MISSION_READY_IMU;
    }
    if (hball_motor_monitor_feedback_fresh(
            &g_hball_motor, now_ms, HBALL_MISSION_READY_SENSOR_FRESH_MS)
        || hball_motor_monitor_motion_parameters_fresh(
            &g_hball_motor,
            now_ms,
            HBALL_RS00_MOTION_PARAMETER_FRESH_MS)
        || hball_motor_monitor_parameters_fresh(
            &g_hball_motor,
            now_ms,
            HBALL_RS00_MOTION_PARAMETER_FRESH_MS))
    {
        ready |= HBALL_MISSION_READY_RS00_LINK;
    }
#if HBALL_INTEGRATED_SHADOW
    {
        hball_sensor_snapshot_t snapshot;

        if (hball_m33_inputs_get_snapshot(&snapshot)
            && ((snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) != 0U))
        {
            ready |= HBALL_MISSION_READY_PI_USB;
            ready |= HBALL_MISSION_READY_VISION;
        }
    }
#endif
    return ready;
}

static void hball_mission_tick(rt_uint32_t now_ms)
{
    hball_mission_status_t status;
    hball_mission_can_frame_t frame;

    hball_mission_arbiter_update_ready(
        &g_hball_mission_arbiter,
        hball_mission_ready_mask(now_ms),
        now_ms
    );
    if ((rt_uint32_t)(now_ms - g_hball_mission_last_status_ms)
        < HBALL_MISSION_STATUS_PERIOD_MS)
    {
        return;
    }
    g_hball_mission_last_status_ms = now_ms;
    if (!hball_mission_arbiter_make_status(
            &g_hball_mission_arbiter, &status)
        || !hball_mission_encode_status(&status, &frame))
    {
        return;
    }
    if (hball_send_mission_frame(&frame) == RT_EOK)
    {
        g_hball_mission_status_tx++;
    }
    else
    {
        g_hball_mission_status_tx_failure++;
    }
}

#if HBALL_RS00_READBACK_TX_ENABLED
static void hball_poll_rs00_readback(rt_uint32_t now_ms)
{
    hball_can_frame_t frame;
    rt_err_t result;
    const uint16_t index = g_hball_rs00_readback_indexes[
        g_hball_rs00_readback_next
    ];

    (void)hball_motor_monitor_expire_parameter_request(
        &g_hball_motor, now_ms, HBALL_RS00_READBACK_TIMEOUT_MS
    );
    if (g_hball_motor.parameter_pending
        || ((rt_uint32_t)(now_ms - g_hball_rs00_readback_last_ms)
            < HBALL_RS00_READBACK_PERIOD_MS))
    {
        return;
    }
    g_hball_rs00_readback_last_ms = now_ms;
    if (!hball_motor_monitor_make_parameter_read(
            &g_hball_motor, index, now_ms, &frame))
    {
        return;
    }
    result = hball_send_can_frame(&frame);
    if (result == RT_EOK)
    {
        g_hball_rs00_readback_next = (rt_uint8_t)(
            (g_hball_rs00_readback_next + 1U)
            % (sizeof(g_hball_rs00_readback_indexes)
                / sizeof(g_hball_rs00_readback_indexes[0]))
        );
    }
    else
    {
        g_hball_motor.parameter_pending = false;
        g_hball_motor.pending_parameter_index = 0U;
    }
}
#endif

static rt_err_t hball_send_probe5_once(void)
{
    hball_can_frame_t frame;
    rt_err_t result;

    if (!g_hball_can_ready)
    {
        rt_kprintf("[hball-m33] probe5 rejected: CAN is not ready\n");
        return -RT_ERROR;
    }
    if (!hball_motor_monitor_make_probe(&g_hball_motor, &frame))
    {
        return -RT_ERROR;
    }

    result = hball_send_can_frame(&frame);
    if (result != RT_EOK)
    {
        g_hball_motor.probe_pending = false;
    }
    rt_kprintf(
        "[hball-m33] probe5 tx id=0x%08lx dlc=%u ret=%d (read-only Get_ID)\n",
        (unsigned long)frame.id,
        (unsigned int)frame.dlc,
        result
    );
    return result;
}

#if HBALL_RS00_MOTION_TX_ENABLED
static rt_err_t hball_motion_send_frame(
    const hball_can_frame_t *frame, const char *label
)
{
    const rt_err_t result = hball_send_can_frame(frame);

    if (result == RT_EOK)
    {
        g_hball_motion_tx_total++;
    }
    if (label != RT_NULL)
    {
        rt_kprintf(
            "[hball-motion] tx %s id=0x%08lx ret=%d\n",
            label,
            (unsigned long)((frame != RT_NULL) ? frame->id : 0U),
            result
        );
    }
    return result;
}

static void hball_motion_stop_now(
    hball_rs00_bench_stop_reason_t reason,
    rt_uint32_t now_ms,
    rt_bool_t automatic
)
{
    hball_can_frame_t frame;
    rt_err_t result = -RT_ERROR;

    if (hball_rs00_control_make_stop(
            HBALL_RS00_MOTOR_ID, false, &frame))
    {
        result = hball_motion_send_frame(&frame, "stop");
    }
    if (result != RT_EOK)
    {
        reason = HBALL_RS00_BENCH_STOP_TX_FAILURE;
    }
    hball_rs00_bench_mark_stopped(&g_hball_motion, reason, now_ms);
#if HBALL_INTEGRATED_SHADOW
    g_hball_ball_active = RT_FALSE;
    g_hball_ball_q3_leveling = RT_FALSE;
    g_hball_ball_q3_leveling_first_target = RT_FALSE;
#endif
    g_hball_motion_prepare_stage = 0U;
    g_hball_motor.parameter_pending = false;
    g_hball_motor.pending_parameter_index = 0U;
    if (automatic)
    {
        g_hball_motion_auto_stop_total++;
    }
    rt_kprintf(
        "[hball-motion] STOPPED reason=%u automatic=%d; power-cut remains emergency stop\n",
        (unsigned int)reason,
        (int)automatic
    );
}

static void hball_motion_reject(const char *operation, const char *reason)
{
    g_hball_motion_reject_total++;
    rt_kprintf(
        "[hball-motion] reject %s: %s state=%s\n",
        operation,
        reason,
        hball_rs00_bench_state_name(g_hball_motion.state)
    );
}

static void hball_motion_begin_prepare(rt_uint32_t now_ms)
{
    const hball_motor_parameters_t *parameters = &g_hball_motor.parameters;

    if (!g_hball_can_ready)
    {
        hball_motion_reject("prepare", "CAN not ready");
        return;
    }
    if (!hball_motor_monitor_parameters_fresh(
            &g_hball_motor,
            now_ms,
            HBALL_RS00_MOTION_PARAMETER_FRESH_MS))
    {
        hball_motion_reject("prepare", "RS00 readback incomplete or stale");
        return;
    }
    if ((parameters->vbus_v < HBALL_RS00_MOTION_VBUS_MIN_V)
        || (parameters->vbus_v > HBALL_RS00_MOTION_VBUS_MAX_V))
    {
        hball_motion_reject("prepare", "VBUS outside 18..26 V");
        return;
    }
    if (fabsf(parameters->mech_velocity_rad_s)
        > HBALL_RS00_MOTION_STATIONARY_RAD_S)
    {
        hball_motion_reject("prepare", "motor is not stationary");
        return;
    }
    if (g_hball_motor.feedback_valid
        && hball_motor_monitor_feedback_fresh(
            &g_hball_motor,
            now_ms,
            HBALL_RS00_MOTION_PARAMETER_FRESH_MS)
        && (g_hball_motor.feedback.fault_summary != 0U))
    {
        hball_motion_reject("prepare", "motor fault is active");
        return;
    }
    if (!hball_rs00_bench_begin_prepare(
            &g_hball_motion, parameters->mech_position_rad, now_ms))
    {
        hball_motion_reject("prepare", "state or hold position invalid");
        return;
    }
    g_hball_motion_prepare_stage = 0U;
    g_hball_motion_next_tx_ms = now_ms;
    rt_kprintf(
        "[hball-motion] PREPARING hold_mrad=%ld speed_mrad_s=%ld current_ma=%ld loc_kp=%ld vbus_mv=%ld\n",
        (long)(g_hball_motion.initial_position_rad * 1000.0F),
        (long)(HBALL_RS00_BENCH_SPEED_LIMIT_RAD_S * 1000.0F),
        (long)(HBALL_RS00_BENCH_CURRENT_LIMIT_A * 1000.0F),
        (long)HBALL_RS00_BENCH_POSITION_KP,
        (long)(parameters->vbus_v * 1000.0F)
    );
}

static void hball_motion_prepare_tick(rt_uint32_t now_ms)
{
    hball_can_frame_t frame;
    rt_bool_t encoded = RT_FALSE;
    const char *label = "invalid";

    if (g_hball_motion.state != HBALL_RS00_BENCH_PREPARING)
    {
        return;
    }
    if (g_hball_motion_prepare_stage >= 7U)
    {
        if (g_hball_motor.parameter_rx_total
            > g_hball_motion_mode_read_rx_baseline)
        {
            if (hball_rs00_bench_mark_prepared(
                    &g_hball_motion,
                    g_hball_motor.parameters.run_mode,
                    now_ms))
            {
                rt_kprintf(
                    "[hball-motion] PREPARED verified run_mode=%u; motor remains disabled\n",
                    (unsigned int)g_hball_motor.parameters.run_mode
                );
            }
            return;
        }
        if ((rt_uint32_t)(now_ms - g_hball_motion_mode_read_ms)
            >= HBALL_RS00_MOTION_PREPARE_TIMEOUT_MS)
        {
            hball_rs00_bench_trip(
                &g_hball_motion,
                HBALL_RS00_BENCH_STOP_MODE_TIMEOUT,
                now_ms
            );
        }
        return;
    }
    if ((rt_int32_t)(now_ms - g_hball_motion_next_tx_ms) < 0)
    {
        return;
    }

    switch (g_hball_motion_prepare_stage)
    {
    case 0U:
        encoded = hball_rs00_control_make_stop(
            HBALL_RS00_MOTOR_ID, false, &frame
        );
        label = "prepare-stop";
        break;
    case 1U:
        encoded = hball_rs00_control_make_csp_mode(
            HBALL_RS00_MOTOR_ID, &frame
        );
        label = "run-mode-csp";
        break;
    case 2U:
        encoded = hball_rs00_control_make_speed_limit(
            HBALL_RS00_MOTOR_ID,
            HBALL_RS00_BENCH_SPEED_LIMIT_RAD_S,
            &frame
        );
        label = "speed-limit";
        break;
    case 3U:
        encoded = hball_rs00_control_make_current_limit(
            HBALL_RS00_MOTOR_ID,
            HBALL_RS00_BENCH_CURRENT_LIMIT_A,
            &frame
        );
        label = "current-limit";
        break;
    case 4U:
        encoded = hball_rs00_control_make_position_kp(
            HBALL_RS00_MOTOR_ID,
            HBALL_RS00_BENCH_POSITION_KP,
            &frame
        );
        label = "position-kp";
        break;
    case 5U:
        encoded = hball_rs00_control_make_position_reference(
            HBALL_RS00_MOTOR_ID,
            g_hball_motion.initial_position_rad,
            &frame
        );
        label = "hold-position";
        break;
    default:
        g_hball_motor.parameter_pending = false;
        g_hball_motor.pending_parameter_index = 0U;
        g_hball_motion_mode_read_rx_baseline =
            g_hball_motor.parameter_rx_total;
        encoded = hball_motor_monitor_make_parameter_read(
            &g_hball_motor,
            HBALL_RS00_INDEX_RUN_MODE,
            now_ms,
            &frame
        );
        g_hball_motion_mode_read_ms = now_ms;
        label = "verify-run-mode";
        break;
    }
    if (!encoded || (hball_motion_send_frame(&frame, label) != RT_EOK))
    {
        hball_rs00_bench_trip(
            &g_hball_motion, HBALL_RS00_BENCH_STOP_TX_FAILURE, now_ms
        );
        return;
    }
    g_hball_motion_prepare_stage++;
    g_hball_motion_next_tx_ms = now_ms + 2U;
}

static rt_bool_t hball_motion_parameter_fresh(
    rt_uint8_t valid_flag, rt_uint8_t slot, rt_uint32_t now_ms
)
{
    return ((g_hball_motor.parameters.valid_flags & valid_flag) != 0U)
        && ((rt_uint32_t)(now_ms
            - g_hball_motor.parameters.last_update_ms[slot])
            <= HBALL_RS00_MOTION_READBACK_FRESH_MS);
}

static void hball_motion_poll_readback(rt_uint32_t now_ms)
{
    static const uint16_t indexes[] = {
        HBALL_RS00_INDEX_MECH_POSITION,
        HBALL_RS00_INDEX_MECH_VELOCITY,
    };
    hball_can_frame_t frame;

    if ((g_hball_motion.state != HBALL_RS00_BENCH_ARMED)
        && (g_hball_motion.state != HBALL_RS00_BENCH_SMALL_STEP)
        && (g_hball_motion.state != HBALL_RS00_BENCH_RETURNING))
    {
        return;
    }
    (void)hball_motor_monitor_expire_parameter_request(
        &g_hball_motor, now_ms, HBALL_RS00_READBACK_TIMEOUT_MS
    );
    if (g_hball_motor.parameter_pending
        || ((rt_uint32_t)(now_ms - g_hball_motion_readback_last_ms)
            < HBALL_RS00_MOTION_READBACK_PERIOD_MS))
    {
        return;
    }
    g_hball_motion_readback_last_ms = now_ms;
    if (!hball_motor_monitor_make_parameter_read(
            &g_hball_motor,
            indexes[g_hball_motion_readback_next],
            now_ms,
            &frame))
    {
        return;
    }
    if (hball_send_can_frame(&frame) != RT_EOK)
    {
        g_hball_motor.parameter_pending = false;
        g_hball_motor.pending_parameter_index = 0U;
        hball_rs00_bench_trip(
            &g_hball_motion, HBALL_RS00_BENCH_STOP_TX_FAILURE, now_ms
        );
        return;
    }
    g_hball_motion_tx_total++;
    g_hball_motion_readback_tx_total++;
    g_hball_motion_readback_next = (rt_uint8_t)(
        (g_hball_motion_readback_next + 1U)
        % (sizeof(indexes) / sizeof(indexes[0]))
    );
}

static void hball_motion_arm(rt_uint32_t now_ms)
{
    hball_can_frame_t frame;

    if ((g_hball_motion.state != HBALL_RS00_BENCH_PREPARED)
        || (g_hball_motor.parameters.run_mode != HBALL_RS00_CSP_MODE))
    {
        hball_motion_reject("arm", "CSP preparation not verified");
        return;
    }
    if (!hball_rs00_control_make_position_reference(
            HBALL_RS00_MOTOR_ID,
            g_hball_motion.initial_position_rad,
            &frame)
        || (hball_motion_send_frame(&frame, "arm-hold-position") != RT_EOK)
        || !hball_rs00_control_make_enable(HBALL_RS00_MOTOR_ID, &frame)
        || (hball_motion_send_frame(&frame, "enable") != RT_EOK)
        || !hball_rs00_bench_begin_arm(&g_hball_motion, now_ms))
    {
        hball_rs00_bench_trip(
            &g_hball_motion, HBALL_RS00_BENCH_STOP_TX_FAILURE, now_ms
        );
        return;
    }
    rt_kprintf(
        "[hball-motion] ARMING: waiting <=%u ms for new 0x02 feedback\n",
        (unsigned int)HBALL_RS00_BENCH_FEEDBACK_TIMEOUT_MS
    );
}

static void hball_motion_step(float step_rad, rt_uint32_t now_ms)
{
    hball_can_frame_t frame;
    float target_rad;

    if (!hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_POSITION,
            HBALL_RS00_PARAMETER_SLOT_MECH_POSITION,
            now_ms)
        || !hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_VELOCITY,
            HBALL_RS00_PARAMETER_SLOT_MECH_VELOCITY,
            now_ms)
        || (hball_motor_monitor_feedback_fresh(
                &g_hball_motor,
                now_ms,
                HBALL_RS00_BENCH_FEEDBACK_TIMEOUT_MS)
            && (g_hball_motor.feedback.fault_summary != 0U)))
    {
        hball_motion_reject("step", "position/velocity stale or faulted");
        hball_rs00_bench_trip(
            &g_hball_motion,
            HBALL_RS00_BENCH_STOP_SENSOR_INVALID,
            now_ms
        );
        return;
    }
    if (!hball_rs00_bench_make_step(
            &g_hball_motion, step_rad, now_ms, &target_rad))
    {
        hball_motion_reject("step", "not ARMED or outside tiny envelope");
        return;
    }
    if (!hball_rs00_control_make_position_reference(
            HBALL_RS00_MOTOR_ID, target_rad, &frame)
        || (hball_motion_send_frame(&frame, "small-step") != RT_EOK))
    {
        hball_rs00_bench_trip(
            &g_hball_motion, HBALL_RS00_BENCH_STOP_TX_FAILURE, now_ms
        );
        return;
    }
    g_hball_step_trace_start_ms = now_ms;
    g_hball_step_trace_last_position_ms = now_ms;
    g_hball_step_trace_count = 0U;
    g_hball_step_trace_active = RT_TRUE;
    rt_kprintf(
        "[hball-motion] SMALL_STEP delta_mrad=%ld target_mrad=%ld\n",
        (long)(step_rad * 1000.0F),
        (long)(target_rad * 1000.0F)
    );
}

static void hball_motion_return(rt_uint32_t now_ms)
{
    hball_can_frame_t frame;
    float target_rad;

    if (!hball_rs00_bench_make_return(
            &g_hball_motion, now_ms, &target_rad))
    {
        hball_motion_reject("return", "no armed small-step to return");
        return;
    }
    if (!hball_rs00_control_make_position_reference(
            HBALL_RS00_MOTOR_ID, target_rad, &frame)
        || (hball_motion_send_frame(&frame, "return") != RT_EOK))
    {
        hball_rs00_bench_trip(
            &g_hball_motion, HBALL_RS00_BENCH_STOP_TX_FAILURE, now_ms
        );
        return;
    }
    rt_kprintf(
        "[hball-motion] RETURNING target_mrad=%ld; automatic stop follows\n",
        (long)(target_rad * 1000.0F)
    );
}

static rt_err_t hball_motion_queue_request(
    hball_motion_request_t request, float step_rad
)
{
    rt_err_t result = -RT_EBUSY;

    if (g_hball_motion_mutex == RT_NULL)
    {
        return -RT_ERROR;
    }
    if (rt_mutex_take(g_hball_motion_mutex, RT_WAITING_FOREVER) != RT_EOK)
    {
        return -RT_ERROR;
    }
    if ((g_hball_motion_request == HBALL_MOTION_REQUEST_NONE)
        || (request == HBALL_MOTION_REQUEST_STOP))
    {
        g_hball_motion_request = request;
        g_hball_motion_request_step_rad = step_rad;
        result = RT_EOK;
    }
    rt_mutex_release(g_hball_motion_mutex);
    return result;
}

static hball_motion_request_t hball_motion_take_request(float *step_rad)
{
    hball_motion_request_t request = HBALL_MOTION_REQUEST_NONE;

    if ((g_hball_motion_mutex == RT_NULL)
        || (rt_mutex_take(g_hball_motion_mutex, 0U) != RT_EOK))
    {
        return request;
    }
    request = g_hball_motion_request;
    *step_rad = g_hball_motion_request_step_rad;
    g_hball_motion_request = HBALL_MOTION_REQUEST_NONE;
    g_hball_motion_request_step_rad = 0.0F;
    rt_mutex_release(g_hball_motion_mutex);
    return request;
}

#if HBALL_RS00_MOTION_TX_ENABLED && HBALL_INTEGRATED_SHADOW
static void hball_mission_action_tick(rt_uint32_t now_ms)
{
    const rt_uint8_t state = g_hball_mission_arbiter.global_state;
    const rt_uint8_t mission = g_hball_mission_arbiter.mission_id;
    const rt_bool_t q456 = hball_m33_q456_is_mission(mission)
        ? RT_TRUE : RT_FALSE;
    float q456_target_m = 0.0F;
    rt_bool_t q456_target_ready = RT_FALSE;

    if (!g_hball_mission_arbiter.context_valid)
    {
        return;
    }
    if (((state == HBALL_MISSION_STATE_CONTROLLED_ABORT)
         || (state == HBALL_MISSION_STATE_FAULT_LATCHED))
        && g_hball_ball_active)
    {
        hball_motion_stop_now(
            HBALL_RS00_BENCH_STOP_MANUAL, now_ms, RT_TRUE
        );
        g_hball_ball_control_epoch = 0U;
        g_hball_ball_control_mission = 0U;
        return;
    }
    if (g_hball_ball_active
        && (g_hball_ball_control_epoch != 0U)
        && ((g_hball_ball_control_epoch
                != g_hball_mission_arbiter.epoch)
            || (g_hball_ball_control_mission != mission)))
    {
        rt_kprintf(
            "[hball-mission] release old control epoch=%u q=%u for epoch=%u q=%u\n",
            (unsigned int)g_hball_ball_control_epoch,
            (unsigned int)g_hball_ball_control_mission,
            (unsigned int)g_hball_mission_arbiter.epoch,
            (unsigned int)mission
        );
        hball_motion_stop_now(
            HBALL_RS00_BENCH_STOP_MANUAL, now_ms, RT_TRUE
        );
        g_hball_ball_control_epoch = 0U;
        g_hball_ball_control_mission = 0U;
        return;
    }
    if (q456)
    {
        hball_sensor_snapshot_t snapshot;

        (void)hball_m33_q456_sync_context(
            &g_hball_q456_runtime,
            g_hball_mission_arbiter.epoch,
            mission
        );
        if (state == HBALL_MISSION_STATE_PREPARING)
        {
            (void)hball_m33_q456_prepare(&g_hball_q456_runtime);
        }
        if (hball_m33_inputs_get_snapshot(&snapshot))
        {
            hball_m33_q456_observe_vision(
                &g_hball_q456_runtime,
                snapshot.vision_sequence,
                snapshot.ball_position_m,
                (snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) != 0U
            );
        }
        if (state == HBALL_MISSION_STATE_START_PENDING)
        {
            q456_target_ready = hball_m33_q456_start_target(
                &g_hball_q456_runtime, &q456_target_m
            ) ? RT_TRUE : RT_FALSE;
        }
    }
    if (state == HBALL_MISSION_STATE_START_PENDING)
    {
        if (g_hball_ball_active
            && (g_hball_ball_control_epoch
                == g_hball_mission_arbiter.epoch)
            && (g_hball_ball_control_mission == mission))
        {
            if (q456 && !q456_target_ready)
            {
                return;
            }
            if (q456)
            {
                g_hball_ball_target_m = q456_target_m;
                g_hball_ball_start_ms =
                    g_hball_mission_arbiter.start_accept_time_ms;
                g_hball_ball_last_step_ms = now_ms;
                g_hball_ball_sensor_invalid_since_ms = 0U;
                g_hball_ball_vision_lost = RT_FALSE;
                g_hball_ball_last_tx_ms = now_ms;
                g_hball_ball_settle_since_ms = 0U;
                g_hball_ball_phase =
                    (mission == HBALL_MISSION_Q6_HOLD_POSITION_LAP)
                        ? 6U : 5U;
                g_hball_ball_mode =
                    (mission == HBALL_MISSION_Q6_HOLD_POSITION_LAP)
                        ? 3U : 2U;
                g_hball_ball_q3_passed = RT_FALSE;
                g_hball_ball_pipeline.controller.integral_error_m_s = 0.0F;
                g_hball_ball_position_integral = 0.0F;
                g_hball_ball_previous_error_m = 0.0F;
                g_hball_ball_pid_static_boost_active = RT_FALSE;
                g_hball_ball_max_abs_error_m = 0.0F;
                g_hball_ball_error_violation_total = 0U;
                g_hball_ball_settle_hold = RT_FALSE;
            }
            (void)hball_mission_arbiter_mark_running(
                &g_hball_mission_arbiter
            );
            if (q456)
            {
                (void)hball_m33_q456_mark_running(
                    &g_hball_q456_runtime,
                    g_hball_mission_arbiter.start_accept_time_ms
                );
            }
            return;
        }
        if (g_hball_motion.state == HBALL_RS00_BENCH_PREPARED)
        {
            hball_motion_arm(now_ms);
            return;
        }
        if (g_hball_motion.state == HBALL_RS00_BENCH_ARMED)
        {
            int result = -RT_ERROR;

            if (mission == HBALL_MISSION_Q3_BALL_SEQUENCE)
            {
                result = hball_q3_start_common();
            }
            else if (q456 && q456_target_ready)
            {
                result = hball_hold_start_common(
                    mission == HBALL_MISSION_Q6_HOLD_POSITION_LAP
                        ? 3U : 2U,
                    q456_target_m,
                    mission == HBALL_MISSION_Q6_HOLD_POSITION_LAP
                        ? "q6-latched" : "q45-center"
                );
            }
            if (result == RT_EOK)
            {
                g_hball_ball_control_epoch =
                    g_hball_mission_arbiter.epoch;
                g_hball_ball_control_mission = mission;
                (void)hball_mission_arbiter_mark_running(
                    &g_hball_mission_arbiter
                );
                if (q456)
                {
                    (void)hball_m33_q456_mark_running(
                        &g_hball_q456_runtime,
                        g_hball_mission_arbiter.start_accept_time_ms
                    );
                }
            }
            else if ((mission == HBALL_MISSION_Q3_BALL_SEQUENCE)
                || (q456 && !q456_target_ready))
            {
                return;
            }
            else
            {
                (void)hball_mission_arbiter_mark_aborted(
                    &g_hball_mission_arbiter,
                    HBALL_MISSION_REASON_LOCAL_FAULT
                );
            }
            return;
        }
        if (((g_hball_motion.state == HBALL_RS00_BENCH_SAFE)
             || (g_hball_motion.state == HBALL_RS00_BENCH_STOPPED))
            && ((rt_uint32_t)(now_ms - g_hball_mission_action_last_ms)
                >= 200U))
        {
            g_hball_mission_action_last_ms = now_ms;
            hball_motion_begin_prepare(now_ms);
        }
        return;
    }
    if (state != HBALL_MISSION_STATE_RUNNING)
    {
        return;
    }
    if ((mission == HBALL_MISSION_Q3_BALL_SEQUENCE)
        && g_hball_ball_q3_passed)
    {
        (void)hball_mission_arbiter_mark_completed(
            &g_hball_mission_arbiter
        );
        return;
    }
    if ((mission == HBALL_MISSION_Q3_BALL_SEQUENCE)
        && !g_hball_ball_active
        && (g_hball_ball_phase == 4U))
    {
        (void)hball_mission_arbiter_mark_aborted(
            &g_hball_mission_arbiter,
            HBALL_MISSION_REASON_DEADLINE
        );
        return;
    }
    if ((mission == HBALL_MISSION_Q3_BALL_SEQUENCE)
        && !g_hball_ball_active
        && (g_hball_motion.state == HBALL_RS00_BENCH_STOPPED))
    {
        const rt_uint8_t reason =
            (g_hball_motion.stop_reason
                == HBALL_RS00_BENCH_STOP_SENSOR_INVALID)
                ? HBALL_MISSION_REASON_LINK_FAULT
                : HBALL_MISSION_REASON_LOCAL_FAULT;

        (void)hball_mission_arbiter_mark_aborted(
            &g_hball_mission_arbiter, reason
        );
        return;
    }
    if (q456)
    {
        const hball_q456_outcome_t outcome = hball_m33_q456_step(
            &g_hball_q456_runtime,
            now_ms,
            g_hball_mission_chassis.epoch,
            g_hball_mission_chassis.event_flags,
            !g_hball_ball_active
        );

        if (outcome == HBALL_Q456_OUTCOME_COMPLETED)
        {
            (void)hball_mission_arbiter_mark_completed(
                &g_hball_mission_arbiter
            );
        }
        else if (outcome == HBALL_Q456_OUTCOME_DEADLINE)
        {
            (void)hball_mission_arbiter_mark_aborted(
                &g_hball_mission_arbiter,
                HBALL_MISSION_REASON_DEADLINE
            );
        }
        else if (outcome == HBALL_Q456_OUTCOME_CONTROL_FAULT)
        {
            (void)hball_mission_arbiter_mark_aborted(
                &g_hball_mission_arbiter,
                HBALL_MISSION_REASON_LOCAL_FAULT
            );
        }
    }
}
#endif

#if HBALL_INTEGRATED_SHADOW
static void hball_ball_submit_actual_log(
    const hball_sensor_snapshot_t *snapshot, rt_uint32_t now_ms
)
{
    hball_log_record_t record;

    if ((snapshot == RT_NULL)
        || ((rt_uint32_t)(now_ms - g_hball_ball_last_log_ms) < 20U))
    {
        return;
    }
    g_hball_ball_last_log_ms = now_ms;
    rt_memset(&record, 0, sizeof(record));
    record.sequence = g_hball_ball_log_sequence++;
    record.produced_time_ms = now_ms;
    record.sensor_sequence = snapshot->sequence;
    record.controller_steps = g_hball_ball_tx_total;
    record.vision_sequence = snapshot->vision_sequence;
    record.sensor_valid_flags = snapshot->valid_flags;
    record.control_mode = (rt_uint16_t)(UINT16_C(0x0100)
        | g_hball_ball_mode);
    record.guard_reason = g_hball_ball_phase;
    record.status_flags = HBALL_LOG_STATUS_Q3_ACTUAL
        | (g_hball_ball_active ? HBALL_LOG_STATUS_CONTROL_ACTIVE : 0U)
        | (g_hball_ball_q3_passed ? HBALL_LOG_STATUS_Q3_PASSED : 0U)
        | (g_hball_ball_q3_leveling
            ? HBALL_LOG_STATUS_Q3_LEVELING : 0U);
    record.ball_position_m = snapshot->ball_position_m;
    record.estimated_position_m =
        g_hball_ball_output.estimated_position_m;
    record.estimated_velocity_mps =
        g_hball_ball_output.estimated_velocity_mps;
    /* Q3 actual frames repurpose this V1 slot as target_position_m. */
    record.estimated_disturbance_mps2 = g_hball_ball_target_m;
    record.pipe_target_rad = g_hball_ball_output.shadow_command_rad;
    record.motor_angle_rad = g_hball_motor.parameters.mech_position_rad;
    record.motor_velocity_rad_s =
        g_hball_motor.parameters.mech_velocity_rad_s;
    record.longitudinal_accel_mps2 =
        snapshot->longitudinal_accel_mps2;
    record.body_pitch_rad = snapshot->body_pitch_rad;
    (void)hball_usb_telemetry_submit(&record);
}

static void hball_ball_commission_tick(rt_uint32_t now_ms)
{
    hball_sensor_snapshot_t snapshot;
    hball_can_frame_t frame;
    float pipe_command_rad;
    float motor_offset_rad;
    float motor_target_rad;
    float position_error_m;
    float pipe_limit_rad;
    rt_uint8_t invalid_mask = 0U;
    rt_uint8_t stop_invalid_mask;
    rt_bool_t vision_stale;
    const rt_bool_t settle_hold =
        (g_hball_ball_mode != 1U)
        && (g_hball_mission_arbiter.global_state
            == HBALL_MISSION_STATE_COMPLETED);

    if (!g_hball_ball_active
        || ((rt_uint32_t)(now_ms - g_hball_ball_last_step_ms)
            < HBALL_BALL_CONTROL_PERIOD_MS))
    {
        return;
    }
    g_hball_ball_last_step_ms = now_ms;
    rt_memset(&snapshot, 0, sizeof(snapshot));
    if (!hball_m33_inputs_get_snapshot(&snapshot))
    {
        invalid_mask |= UINT8_C(1) << 0;
    }
    if ((snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) == 0U)
    {
        invalid_mask |= UINT8_C(1) << 1;
    }
    if (snapshot.vision_receive_age_ms > HBALL_BALL_VISION_HOLD_MS)
    {
        invalid_mask |= UINT8_C(1) << 2;
    }
    if (!hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_POSITION,
            HBALL_RS00_PARAMETER_SLOT_MECH_POSITION,
            now_ms))
    {
        invalid_mask |= UINT8_C(1) << 3;
    }
    if (!hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_VELOCITY,
            HBALL_RS00_PARAMETER_SLOT_MECH_VELOCITY,
            now_ms))
    {
        invalid_mask |= UINT8_C(1) << 4;
    }
    vision_stale = (invalid_mask & UINT8_C(0x06)) != 0U;
    stop_invalid_mask = invalid_mask & UINT8_C(0x19);
    if ((stop_invalid_mask == 0U) && !vision_stale
        && (fabsf(snapshot.ball_position_m)
            >= HBALL_BALL_COMMISSION_POSITION_LIMIT_M))
    {
        rt_kprintf(
            "[hball-q3] abort position limit x_mm=%ld\n",
            (long)(snapshot.ball_position_m * 1000.0F)
        );
        hball_motion_stop_now(
            HBALL_RS00_BENCH_STOP_SENSOR_INVALID, now_ms, RT_TRUE
        );
        return;
    }
    if (stop_invalid_mask != 0U)
    {
        if (g_hball_ball_sensor_invalid_since_ms == 0U)
        {
            g_hball_ball_sensor_invalid_since_ms = now_ms;
            return;
        }
        if ((rt_uint32_t)(now_ms
                - g_hball_ball_sensor_invalid_since_ms)
            < HBALL_BALL_SENSOR_GRACE_MS)
        {
            return;
        }
        rt_kprintf(
            "[hball-q3] abort stale mask=0x%02x vision_age=%lu "
            "motor_pos_age=%lu motor_vel_age=%lu\n",
            (unsigned int)stop_invalid_mask,
            (unsigned long)snapshot.vision_receive_age_ms,
            (unsigned long)(now_ms
                - g_hball_motor.parameters.last_update_ms[
                    HBALL_RS00_PARAMETER_SLOT_MECH_POSITION]),
            (unsigned long)(now_ms
                - g_hball_motor.parameters.last_update_ms[
                    HBALL_RS00_PARAMETER_SLOT_MECH_VELOCITY])
        );
        hball_motion_stop_now(
            HBALL_RS00_BENCH_STOP_SENSOR_INVALID, now_ms, RT_TRUE
        );
        return;
    }
    g_hball_ball_sensor_invalid_since_ms = 0U;
    snapshot.valid_flags |= HBALL_SENSOR_VALID_MOTOR;
    snapshot.motor_age_ms = 0U;
    snapshot.motor_angle_rad =
        g_hball_motor.parameters.mech_position_rad;
    snapshot.motor_velocity_rad_s =
        g_hball_motor.parameters.mech_velocity_rad_s;
    if ((g_hball_ball_mode == 1U) && g_hball_ball_q3_leveling)
    {
        const float max_delta_rad =
            HBALL_BALL_Q3_PRELEVEL_PIPE_RATE_LIMIT_RAD_S
            * HBALL_BALL_CONTROL_DT_S;
        float delta_rad = -g_hball_ball_q3_pipe_command_rad;

        if (g_hball_ball_q3_leveling_first_target)
        {
            delta_rad = 0.0F;
            g_hball_ball_q3_leveling_first_target = RT_FALSE;
        }
        else if (delta_rad > max_delta_rad)
        {
            delta_rad = max_delta_rad;
        }
        else if (delta_rad < -max_delta_rad)
        {
            delta_rad = -max_delta_rad;
        }
        g_hball_ball_q3_pipe_command_rad += delta_rad;
        if (!hball_fourbar_motor_offset(
                &g_hball_ball_pipeline.fourbar,
                g_hball_ball_q3_pipe_command_rad,
                &motor_offset_rad))
        {
            hball_motion_stop_now(
                HBALL_RS00_BENCH_STOP_SENSOR_INVALID, now_ms, RT_TRUE
            );
            return;
        }
        motor_target_rad = g_hball_ball_level_rad
            + HBALL_LINKAGE_MOTOR_DIRECTION_SIGN * motor_offset_rad;
        g_hball_ball_output.shadow_command_rad =
            g_hball_ball_q3_pipe_command_rad;
        g_hball_ball_output.motor_target_rad = motor_target_rad;
        if ((rt_uint32_t)(now_ms - g_hball_ball_last_tx_ms)
            >= HBALL_BALL_COMMISSION_TX_PERIOD_MS)
        {
            g_hball_ball_last_tx_ms = now_ms;
            if (!hball_rs00_control_make_position_reference(
                    HBALL_RS00_MOTOR_ID, motor_target_rad, &frame)
                || (hball_motion_send_frame(&frame, RT_NULL) != RT_EOK))
            {
                hball_motion_stop_now(
                    HBALL_RS00_BENCH_STOP_TX_FAILURE, now_ms, RT_TRUE
                );
                return;
            }
            g_hball_ball_tx_total++;
        }
        g_hball_motion.last_manual_command_ms = now_ms;
        hball_ball_submit_actual_log(&snapshot, now_ms);
        if ((fabsf(g_hball_motor.parameters.mech_position_rad
                    - g_hball_ball_level_rad) <= 0.002F)
            && (fabsf(g_hball_motor.parameters.mech_velocity_rad_s)
                <= 0.05F))
        {
            if (g_hball_ball_settle_since_ms == 0U)
            {
                g_hball_ball_settle_since_ms = now_ms;
            }
            else if ((rt_uint32_t)(now_ms
                    - g_hball_ball_settle_since_ms) >= 100U)
            {
                const float initial_position_m = !vision_stale
                        && isfinite(snapshot.ball_position_m)
                    ? snapshot.ball_position_m : 0.0F;

                hball_control_pipeline_init(
                    &g_hball_ball_pipeline, initial_position_m
                );
                (void)hball_control_pipeline_set_motor_level(
                    &g_hball_ball_pipeline, g_hball_ball_level_rad
                );
                rt_memset(
                    &g_hball_ball_output, 0,
                    sizeof(g_hball_ball_output)
                );
                g_hball_ball_output.motor_target_rad =
                    g_hball_ball_level_rad;
                g_hball_ball_q3_pipe_command_rad = 0.0F;
                g_hball_ball_position_integral = 0.0F;
                g_hball_ball_previous_error_m = 0.0F;
                g_hball_ball_pid_static_boost_active = RT_FALSE;
                g_hball_ball_settle_since_ms = 0U;
                g_hball_ball_start_ms = 0U;
                g_hball_ball_q3_leveling = RT_FALSE;
                rt_kprintf(
                    "[hball-q3] mechanical level ready; recover O\n"
                );
            }
        }
        else
        {
            g_hball_ball_settle_since_ms = 0U;
        }
        return;
    }
    if ((g_hball_ball_mode == 1U) && vision_stale)
    {
        if (!g_hball_ball_vision_lost)
        {
            g_hball_ball_vision_lost = RT_TRUE;
            rt_kprintf(
                "[hball-q3] vision lost age_ms=%lu; holding last CSP target\n",
                (unsigned long)snapshot.vision_receive_age_ms
            );
        }
        if ((rt_uint32_t)(now_ms - g_hball_ball_last_tx_ms)
            >= HBALL_BALL_COMMISSION_TX_PERIOD_MS)
        {
            g_hball_ball_last_tx_ms = now_ms;
            if (!hball_rs00_control_make_position_reference(
                    HBALL_RS00_MOTOR_ID,
                    g_hball_ball_output.motor_target_rad,
                    &frame)
                || (hball_motion_send_frame(&frame, RT_NULL) != RT_EOK))
            {
                hball_motion_stop_now(
                    HBALL_RS00_BENCH_STOP_TX_FAILURE, now_ms, RT_TRUE
                );
                return;
            }
            g_hball_ball_tx_total++;
        }
        g_hball_motion.last_manual_command_ms = now_ms;
        return;
    }
    if ((g_hball_ball_mode == 1U) && g_hball_ball_vision_lost)
    {
        g_hball_ball_vision_lost = RT_FALSE;
        rt_kprintf("[hball-q3] vision reacquired; feedback resumed\n");
    }
    if ((g_hball_ball_mode != 1U) && vision_stale
        && !g_hball_ball_vision_lost)
    {
        g_hball_ball_vision_lost = RT_TRUE;
        rt_kprintf(
            "[hball-hold] vision lost age_ms=%lu; motor stays enabled, returning level\n",
            (unsigned long)snapshot.vision_receive_age_ms
        );
    }
    else if (!vision_stale && g_hball_ball_vision_lost)
    {
        g_hball_ball_vision_lost = RT_FALSE;
        rt_kprintf("[hball-hold] vision reacquired; feedback resumed\n");
    }
    if (settle_hold && !g_hball_ball_settle_hold)
    {
        g_hball_ball_pipeline.controller.integral_error_m_s = 0.0F;
    }
    g_hball_ball_settle_hold = settle_hold;
    if ((g_hball_ball_mode == 1U)
        && (g_hball_ball_phase == 2U)
        && (g_hball_ball_target_m > -0.050F))
    {
        g_hball_ball_target_m -=
            HBALL_BALL_Q3_TARGET_RATE_MPS * HBALL_BALL_CONTROL_DT_S;
        if (g_hball_ball_target_m < -0.050F)
        {
            g_hball_ball_target_m = -0.050F;
        }
    }
    hball_control_pipeline_step(
        &g_hball_ball_pipeline,
        &snapshot,
        HBALL_BALL_CONTROL_DT_S,
        g_hball_ball_target_m,
        &g_hball_ball_output
    );
    position_error_m =
        g_hball_ball_target_m - g_hball_ball_output.estimated_position_m;
    if ((g_hball_ball_mode == 1U) && (g_hball_ball_phase != 4U))
    {
        g_hball_ball_position_integral +=
            position_error_m * HBALL_BALL_CONTROL_DT_S;
        if (g_hball_ball_position_integral
            > HBALL_BALL_PID_INTEGRAL_LIMIT)
        {
            g_hball_ball_position_integral =
                HBALL_BALL_PID_INTEGRAL_LIMIT;
        }
        else if (g_hball_ball_position_integral
            < -HBALL_BALL_PID_INTEGRAL_LIMIT)
        {
                g_hball_ball_position_integral =
                    -HBALL_BALL_PID_INTEGRAL_LIMIT;
        }
        g_hball_ball_previous_error_m = position_error_m;
    }
    if (settle_hold)
    {
        pipe_limit_rad = fabsf(snapshot.ball_position_m) >= 0.080F
            ? g_hball_settle_recovery_limit_rad
            : (fabsf(snapshot.ball_position_m) >= 0.030F
                ? g_hball_settle_capture_limit_rad
                : g_hball_settle_pipe_limit_rad);
    }
    else if (g_hball_ball_mode != 1U)
    {
        pipe_limit_rad = fabsf(snapshot.ball_position_m) >= 0.080F
            ? g_hball_formal_recovery_limit_rad
            : g_hball_formal_pipe_limit_rad;
        if (((g_hball_ball_control_mission == HBALL_MISSION_Q4_A_TO_B)
                || (g_hball_ball_control_mission
                    == HBALL_MISSION_Q5_CENTER_LAP))
            && ((rt_uint32_t)(now_ms - g_hball_ball_start_ms)
                < HBALL_BALL_Q45_STARTUP_LIMIT_MS)
            && (pipe_limit_rad
                < HBALL_BALL_Q45_STARTUP_PIPE_LIMIT_RAD))
        {
            pipe_limit_rad = HBALL_BALL_Q45_STARTUP_PIPE_LIMIT_RAD;
        }
    }
    else
    {
        pipe_limit_rad = fabsf(snapshot.ball_position_m) >= 0.080F
            ? HBALL_BALL_COMMISSION_RECOVERY_LIMIT_RAD
            : HBALL_BALL_COMMISSION_PIPE_LIMIT_RAD;
    }
    if (g_hball_ball_phase == 4U)
    {
        pipe_command_rad = 0.0F;
    }
    else if (g_hball_ball_mode != 1U)
    {
        /* Q4-Q6 use the portable OOSM Kalman + LQI deployment pipeline. */
        pipe_command_rad = g_hball_ball_output.shadow_command_rad;
    }
    else if (g_hball_ball_use_lqi)
    {
        pipe_command_rad =
            HBALL_BALL_LQI_KP * position_error_m
            + HBALL_BALL_LQI_KI * g_hball_ball_position_integral
            - HBALL_BALL_LQI_KD
                * g_hball_ball_output.estimated_velocity_mps;
    }
    else
    {
        pipe_command_rad =
            g_hball_ball_pid_kp * position_error_m
            + g_hball_ball_pid_ki * g_hball_ball_position_integral
            - g_hball_ball_pid_kd
                * g_hball_ball_output.estimated_velocity_mps;
        if (fabsf(position_error_m) <= 0.001F)
        {
            g_hball_ball_pid_static_boost_active = RT_FALSE;
        }
        else if (g_hball_ball_pid_static_boost_active)
        {
            if (fabsf(g_hball_ball_output.estimated_velocity_mps)
                >= HBALL_BALL_PID_BOOST_EXIT_MPS)
            {
                g_hball_ball_pid_static_boost_active = RT_FALSE;
            }
        }
        else if (fabsf(g_hball_ball_output.estimated_velocity_mps)
            <= HBALL_BALL_PID_BOOST_ENTER_MPS)
        {
            g_hball_ball_pid_static_boost_active = RT_TRUE;
        }
        if (g_hball_ball_pid_static_boost_active)
        {
            pipe_command_rad += copysignf(
                g_hball_ball_pid_static_boost_rad, position_error_m
            );
        }
    }
    if (pipe_command_rad > pipe_limit_rad)
    {
        pipe_command_rad = pipe_limit_rad;
    }
    else if (pipe_command_rad < -pipe_limit_rad)
    {
        pipe_command_rad = -pipe_limit_rad;
    }
    if (g_hball_ball_mode == 1U)
    {
        g_hball_ball_q3_pipe_command_rad = pipe_command_rad;
    }
    if (!hball_fourbar_motor_offset(
            &g_hball_ball_pipeline.fourbar,
            pipe_command_rad,
            &motor_offset_rad))
    {
        hball_motion_stop_now(
            HBALL_RS00_BENCH_STOP_SENSOR_INVALID, now_ms, RT_TRUE
        );
        return;
    }
    motor_target_rad = g_hball_ball_level_rad
        + HBALL_LINKAGE_MOTOR_DIRECTION_SIGN * motor_offset_rad;
    g_hball_ball_output.shadow_command_rad = pipe_command_rad;
    g_hball_ball_output.motor_target_rad = motor_target_rad;
    if ((rt_uint32_t)(now_ms - g_hball_ball_last_tx_ms)
        >= HBALL_BALL_COMMISSION_TX_PERIOD_MS)
    {
        g_hball_ball_last_tx_ms = now_ms;
        if (!hball_rs00_control_make_position_reference(
                HBALL_RS00_MOTOR_ID, motor_target_rad, &frame)
            || (hball_motion_send_frame(&frame, RT_NULL) != RT_EOK))
        {
            hball_motion_stop_now(
                HBALL_RS00_BENCH_STOP_TX_FAILURE, now_ms, RT_TRUE
            );
            return;
        }
        g_hball_ball_tx_total++;
    }
    g_hball_motion.last_manual_command_ms = now_ms;
    hball_ball_submit_actual_log(&snapshot, now_ms);
    if (g_hball_ball_phase == 0U)
    {
        if ((fabsf(snapshot.ball_position_m)
                <= HBALL_BALL_Q3_START_WINDOW_M)
            && (fabsf(g_hball_ball_output.estimated_velocity_mps)
                <= 0.020F))
        {
            if (g_hball_ball_settle_since_ms == 0U)
            {
                g_hball_ball_settle_since_ms = now_ms;
            }
            else if ((rt_uint32_t)(now_ms
                    - g_hball_ball_settle_since_ms)
                >= HBALL_BALL_Q3_LEVEL_SETTLE_MS)
            {
                g_hball_ball_phase = 1U;
                g_hball_ball_target_m = 0.050F;
                g_hball_ball_start_ms = now_ms;
                g_hball_ball_position_integral = 0.0F;
                g_hball_ball_previous_error_m = 0.0F;
                g_hball_ball_settle_since_ms = 0U;
                rt_kprintf(
                    "[hball-q3] level ready x_mm=%ld; start +50,-50 deadline=5000 ms\n",
                    (long)(snapshot.ball_position_m * 1000.0F)
                );
            }
        }
        else
        {
            g_hball_ball_settle_since_ms = 0U;
        }
        return;
    }
    if (g_hball_ball_phase == 4U)
    {
        if ((fabsf(g_hball_motor.parameters.mech_position_rad
                    - g_hball_ball_level_rad) <= 0.002F)
            && (fabsf(g_hball_motor.parameters.mech_velocity_rad_s)
                <= 0.05F))
        {
            if (g_hball_ball_settle_since_ms == 0U)
            {
                g_hball_ball_settle_since_ms = now_ms;
            }
            else if ((rt_uint32_t)(now_ms
                    - g_hball_ball_settle_since_ms) >= 50U)
            {
                hball_motion_stop_now(
                    HBALL_RS00_BENCH_STOP_MANUAL, now_ms, RT_TRUE
                );
            }
        }
        else
        {
            g_hball_ball_settle_since_ms = 0U;
        }
        return;
    }
    if (vision_stale)
    {
        g_hball_ball_settle_since_ms = 0U;
        return;
    }
    if (fabsf(position_error_m) > g_hball_ball_max_abs_error_m)
    {
        g_hball_ball_max_abs_error_m = fabsf(position_error_m);
    }
    if (fabsf(position_error_m) > 0.010F)
    {
        g_hball_ball_error_violation_total++;
    }
    position_error_m = snapshot.ball_position_m - g_hball_ball_target_m;
    if ((fabsf(position_error_m) <= 0.010F)
        && ((g_hball_ball_phase != 2U)
            || (g_hball_ball_target_m <= -0.050F))
        && (fabsf(g_hball_ball_output.estimated_velocity_mps) <= 0.020F))
    {
        if (g_hball_ball_settle_since_ms == 0U)
        {
            g_hball_ball_settle_since_ms = now_ms;
        }
    }
    else
    {
        g_hball_ball_settle_since_ms = 0U;
    }
    if ((g_hball_ball_mode == 1U)
        && (g_hball_ball_phase == 1U)
        && (g_hball_ball_settle_since_ms != 0U)
        && ((rt_uint32_t)(now_ms - g_hball_ball_settle_since_ms) >= 150U))
    {
        g_hball_ball_phase = 2U;
        g_hball_ball_position_integral = 0.0F;
        g_hball_ball_previous_error_m = 0.0F;
        g_hball_ball_pid_static_boost_active = RT_FALSE;
        g_hball_ball_settle_since_ms = 0U;
        rt_kprintf("[hball-q3] reached +50 mm; returning to -50 mm\n");
    }
    if ((g_hball_ball_mode == 1U)
        && (g_hball_ball_phase == 2U)
        && (g_hball_ball_settle_since_ms != 0U)
        && ((rt_uint32_t)(now_ms - g_hball_ball_settle_since_ms) >= 300U))
    {
        g_hball_ball_phase = 3U;
        g_hball_ball_q3_passed =
            ((rt_uint32_t)(now_ms - g_hball_ball_start_ms) <= 5000U);
        rt_kprintf(
            "[hball-q3] settled -50 mm elapsed_ms=%lu pass=%d\n",
            (unsigned long)(now_ms - g_hball_ball_start_ms),
            (int)g_hball_ball_q3_passed
        );
    }
    if ((g_hball_ball_mode == 1U)
        && !g_hball_ball_q3_passed
        && (g_hball_ball_phase != 4U)
        && ((rt_uint32_t)(now_ms - g_hball_ball_start_ms) >= 5000U))
    {
        g_hball_ball_phase = 4U;
        g_hball_ball_settle_since_ms = 0U;
        rt_kprintf("[hball-q3] fail 5 s deadline; returning level\n");
    }
    else if ((g_hball_ball_mode != 1U)
        && (g_hball_ball_phase != 4U)
        && (HBALL_BALL_HOLD_TIMEOUT_MS != 0U)
        && ((rt_uint32_t)(now_ms - g_hball_ball_start_ms)
            >= HBALL_BALL_HOLD_TIMEOUT_MS))
    {
        g_hball_ball_phase = 4U;
        g_hball_ball_settle_since_ms = 0U;
    }
}
#endif

static void hball_motion_tick(rt_uint32_t now_ms)
{
    float step_rad = 0.0F;
    const hball_motion_request_t request =
        hball_motion_take_request(&step_rad);

    if (request == HBALL_MOTION_REQUEST_STOP)
    {
        hball_motion_stop_now(
            HBALL_RS00_BENCH_STOP_MANUAL, now_ms, RT_FALSE
        );
        return;
    }
    if (g_hball_motion.state == HBALL_RS00_BENCH_FAULT)
    {
        hball_motion_stop_now(
            g_hball_motion.stop_reason, now_ms, RT_TRUE
        );
        return;
    }

    switch (request)
    {
    case HBALL_MOTION_REQUEST_PROBE:
        if ((g_hball_motion.state == HBALL_RS00_BENCH_SAFE)
            || (g_hball_motion.state == HBALL_RS00_BENCH_STOPPED))
        {
            (void)hball_send_probe5_once();
        }
        else
        {
            hball_motion_reject("probe", "manual motion session active");
        }
        break;
    case HBALL_MOTION_REQUEST_PREPARE:
        hball_motion_begin_prepare(now_ms);
        break;
    case HBALL_MOTION_REQUEST_ARM:
        hball_motion_arm(now_ms);
        break;
    case HBALL_MOTION_REQUEST_STEP:
        hball_motion_step(step_rad, now_ms);
        break;
    case HBALL_MOTION_REQUEST_RETURN:
        hball_motion_return(now_ms);
        break;
    default:
        break;
    }

    hball_motion_prepare_tick(now_ms);
    hball_motion_poll_readback(now_ms);
#if HBALL_INTEGRATED_SHADOW
    hball_ball_commission_tick(now_ms);
#endif
    if ((g_hball_motion.state == HBALL_RS00_BENCH_ARMED)
        || (g_hball_motion.state == HBALL_RS00_BENCH_SMALL_STEP)
        || (g_hball_motion.state == HBALL_RS00_BENCH_RETURNING))
    {
        const rt_bool_t feedback_fresh =
            hball_motor_monitor_feedback_fresh(
                &g_hball_motor,
                now_ms,
                HBALL_RS00_BENCH_FEEDBACK_TIMEOUT_MS);
        const rt_bool_t position_fresh = hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_POSITION,
            HBALL_RS00_PARAMETER_SLOT_MECH_POSITION,
            now_ms);
        const rt_bool_t velocity_fresh = hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_VELOCITY,
            HBALL_RS00_PARAMETER_SLOT_MECH_VELOCITY,
            now_ms);

        if (!feedback_fresh && (!position_fresh || !velocity_fresh))
        {
            hball_rs00_bench_trip(
                &g_hball_motion,
                HBALL_RS00_BENCH_STOP_FEEDBACK_TIMEOUT,
                now_ms
            );
        }
    }
    if ((g_hball_motion.state == HBALL_RS00_BENCH_RETURNING)
        && ((rt_uint32_t)(now_ms - g_hball_motion.state_since_ms) >= 20U)
        && hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_POSITION,
            HBALL_RS00_PARAMETER_SLOT_MECH_POSITION,
            now_ms)
        && hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_VELOCITY,
            HBALL_RS00_PARAMETER_SLOT_MECH_VELOCITY,
            now_ms)
        && (fabsf(g_hball_motor.parameters.mech_position_rad
                - g_hball_motion.initial_position_rad)
            <= HBALL_RS00_MOTION_RETURN_TOLERANCE_RAD)
        && (fabsf(g_hball_motor.parameters.mech_velocity_rad_s)
            <= HBALL_RS00_MOTION_RETURN_VELOCITY_RAD_S))
    {
        if (g_hball_motion_return_settle_since_ms == 0U)
        {
            g_hball_motion_return_settle_since_ms = now_ms;
        }
        else if ((rt_uint32_t)(now_ms
                - g_hball_motion_return_settle_since_ms)
            >= HBALL_RS00_MOTION_RETURN_SETTLE_MS)
        {
            hball_motion_stop_now(
                HBALL_RS00_BENCH_STOP_MANUAL, now_ms, RT_TRUE
            );
            return;
        }
    }
    else
    {
        g_hball_motion_return_settle_since_ms = 0U;
    }
    if ((g_hball_motion.state == HBALL_RS00_BENCH_RETURNING)
        && ((rt_uint32_t)(now_ms - g_hball_motion.state_since_ms)
            >= HBALL_RS00_MOTION_RETURN_TIMEOUT_MS))
    {
        hball_rs00_bench_trip(
            &g_hball_motion,
            HBALL_RS00_BENCH_STOP_RETURN_TIMEOUT,
            now_ms
        );
    }
    (void)hball_rs00_bench_watchdog_expired(&g_hball_motion, now_ms);
    if (g_hball_motion.state == HBALL_RS00_BENCH_FAULT)
    {
        hball_motion_stop_now(
            g_hball_motion.stop_reason, now_ms, RT_TRUE
        );
    }
}
#endif

static void hball_worker_entry(void *parameter)
{
#if HBALL_BENCH_AUTO_PROBE5
    rt_uint32_t boot_ms = hball_now_ms();
    rt_bool_t auto_probe_done = RT_FALSE;
#endif
#if HBALL_PERIODIC_DIAGNOSTICS
    rt_uint32_t last_log_ms = hball_now_ms();
#endif

    RT_UNUSED(parameter);
    while (1)
    {
        rt_uint32_t now_ms;

        hball_poll_can();
        now_ms = hball_now_ms();
        hball_mission_tick(now_ms);
#if HBALL_RS00_MOTION_TX_ENABLED
        hball_motion_tick(now_ms);
#if HBALL_INTEGRATED_SHADOW
        hball_mission_action_tick(now_ms);
#endif
#endif
#if HBALL_RS00_READBACK_TX_ENABLED
#if HBALL_RS00_MOTION_TX_ENABLED
        if ((g_hball_motion.state == HBALL_RS00_BENCH_SAFE)
            || (g_hball_motion.state == HBALL_RS00_BENCH_STOPPED))
#endif
        {
        hball_poll_rs00_readback(now_ms);
        }
#endif
#if HBALL_BENCH_AUTO_PROBE5
        if (!auto_probe_done
            && ((rt_uint32_t)(now_ms - boot_ms) >= HBALL_BENCH_AUTO_PROBE_DELAY_MS))
        {
            auto_probe_done = RT_TRUE;
            (void)hball_send_probe5_once();
        }
#endif
#if HBALL_PERIODIC_DIAGNOSTICS
        if ((rt_uint32_t)(now_ms - last_log_ms) >= HBALL_BENCH_LOG_PERIOD_MS)
        {
            last_log_ms = now_ms;
            rt_kprintf(
                "[hball-m33] alive can_rx=%lu tx_ok=%lu tx_fail=%lu manual_motion_tx=%u ACTUATOR_TX=0\n",
                (unsigned long)g_hball_raw_rx_total,
                (unsigned long)g_hball_tx_success,
                (unsigned long)g_hball_tx_failure,
                (unsigned int)HBALL_RS00_MOTION_TX_ENABLED
            );
        }
#endif
        rt_thread_mdelay(HBALL_BENCH_PERIOD_MS);
    }
}

static void hball_init(void)
{
    rt_err_t result;

    if (g_hball_can_ready)
    {
        rt_kprintf("[hball-m33] CAN already ready\n");
        return;
    }
    ifx_can_direct_set_tx_verbose(RT_TRUE);
    result = ifx_can_direct_init();
    g_hball_can_ready = (result == RT_EOK) ? RT_TRUE : RT_FALSE;
    rt_kprintf(
        "[hball-m33] init version=%s bitrate=%lu result=%d ready=%d\n",
        HBALL_BENCH_VERSION,
        (unsigned long)HBALL_CAN_CLASSIC_BITRATE,
        result,
        (int)g_hball_can_ready
    );
}
MSH_CMD_EXPORT(hball_init, initialize H-ball CAN monitor and manual bench path);

static void hball_status(void)
{
    ifx_can_direct_diag_t diagnostic;
    rt_err_t result;
    rt_uint8_t index;
    const rt_uint32_t can_rate_x10 = hball_rate_meter_hz_x10(
        &g_hball_can_rx_rate, hball_now_ms()
    );

    rt_memset(&diagnostic, 0, sizeof(diagnostic));
    result = ifx_can_direct_get_diag(&diagnostic);
    rt_kprintf(
        "[hball-m33] status version=%s read_only=%u manual_motion_tx=%u can_ready=%d tx=%lu/%lu/%lu raw_rx=%lu can_rate_x10=%lu rx_fifo_depth=%u\n",
        HBALL_BENCH_VERSION,
        (unsigned int)(HBALL_RS00_MOTION_TX_ENABLED ? 0U : 1U),
        (unsigned int)HBALL_RS00_MOTION_TX_ENABLED,
        (int)g_hball_can_ready,
        (unsigned long)g_hball_tx_total,
        (unsigned long)g_hball_tx_success,
        (unsigned long)g_hball_tx_failure,
        (unsigned long)g_hball_raw_rx_total,
        (unsigned long)can_rate_x10,
        (unsigned int)BSP_CANFD0_RX_FIFO0_ELEMENTS
    );
    rt_kprintf(
        "[hball-m33] probe pending=%d valid=%d uid=%08lx%08lx motor_rx=%lu invalid=%lu ignored=%lu\n",
        (int)g_hball_motor.probe_pending,
        (int)g_hball_motor.probe_valid,
        (unsigned long)(g_hball_motor.unique_id >> 32),
        (unsigned long)(g_hball_motor.unique_id & 0xFFFFFFFFULL),
        (unsigned long)g_hball_motor.rx_total,
        (unsigned long)g_hball_motor.rx_invalid,
        (unsigned long)g_hball_motor.rx_ignored
    );
    rt_kprintf(
        "[hball-m33] rs00_readback enabled=%u pending=%d index=0x%04x valid=0x%02x rx=%lu timeout=%lu mode=%u rotation=%d pos_mrad=%ld vel_mrad_s=%ld iq_ma=%ld vbus_mv=%ld\n",
        (unsigned int)HBALL_RS00_READBACK_TX_ENABLED,
        (int)g_hball_motor.parameter_pending,
        (unsigned int)g_hball_motor.pending_parameter_index,
        (unsigned int)g_hball_motor.parameters.valid_flags,
        (unsigned long)g_hball_motor.parameter_rx_total,
        (unsigned long)g_hball_motor.parameter_timeout_total,
        (unsigned int)g_hball_motor.parameters.run_mode,
        (int)g_hball_motor.parameters.rotation,
        (long)(g_hball_motor.parameters.mech_position_rad * 1000.0F),
        (long)(g_hball_motor.parameters.mech_velocity_rad_s * 1000.0F),
        (long)(g_hball_motor.parameters.filtered_iq_a * 1000.0F),
        (long)(g_hball_motor.parameters.vbus_v * 1000.0F)
    );
    rt_kprintf(
        "[hball-m33] rs00_feedback valid=%d age_ms=%lu mode=%u fault=0x%02x pos_mrad=%ld vel_mrad_s=%ld torque_mnm=%ld temp_dc=%ld error_raw=%lu last_error_id=0x%08lx dlc=%u\n",
        (int)g_hball_motor.feedback_valid,
        (unsigned long)(hball_now_ms() - g_hball_motor.last_feedback_ms),
        (unsigned int)g_hball_motor.feedback.mode_state,
        (unsigned int)g_hball_motor.feedback.fault_summary,
        (long)(g_hball_motor.feedback.position_rad * 1000.0F),
        (long)(g_hball_motor.feedback.velocity_rad_s * 1000.0F),
        (long)(g_hball_motor.feedback.torque_nm * 1000.0F),
        (long)(g_hball_motor.feedback.temperature_c * 10.0F),
        (unsigned long)g_hball_motor.error_raw_total,
        (unsigned long)g_hball_motor.last_error_raw_id,
        (unsigned int)g_hball_motor.last_error_raw_dlc
    );
#if HBALL_RS00_MOTION_TX_ENABLED
    rt_kprintf(
        "[hball-motion] state=%s reason=%u initial_mrad=%ld target_mrad=%ld stage=%u manual_tx=%lu motion_readback_tx=%lu reject=%lu auto_stop=%lu limits=%ld_mrad_s/%ld_mA step_max=%ld_mrad envelope=%ld_mrad watchdog_ms=%u\n",
        hball_rs00_bench_state_name(g_hball_motion.state),
        (unsigned int)g_hball_motion.stop_reason,
        (long)(g_hball_motion.initial_position_rad * 1000.0F),
        (long)(g_hball_motion.target_position_rad * 1000.0F),
        (unsigned int)g_hball_motion_prepare_stage,
        (unsigned long)g_hball_motion_tx_total,
        (unsigned long)g_hball_motion_readback_tx_total,
        (unsigned long)g_hball_motion_reject_total,
        (unsigned long)g_hball_motion_auto_stop_total,
        (long)(HBALL_RS00_BENCH_SPEED_LIMIT_RAD_S * 1000.0F),
        (long)(HBALL_RS00_BENCH_CURRENT_LIMIT_A * 1000.0F),
        (long)(HBALL_RS00_BENCH_STEP_MAX_RAD * 1000.0F),
        (long)(HBALL_RS00_BENCH_ENVELOPE_RAD * 1000.0F),
        (unsigned int)HBALL_RS00_BENCH_COMMAND_TIMEOUT_MS
    );
#endif
    rt_kprintf(
        "[hball-m33] msp_rx=%lu invalid=%lu ignored=%lu dup=%lu ooo=%lu gap=%lu reboot=%lu hb=%d accel=%d gyro=%d attitude=%d wheel=%d status=0x%04x\n",
        (unsigned long)g_hball_msp.rx_total,
        (unsigned long)g_hball_msp.rx_invalid,
        (unsigned long)g_hball_msp.rx_ignored,
        (unsigned long)g_hball_msp.rx_duplicate,
        (unsigned long)g_hball_msp.rx_out_of_order,
        (unsigned long)g_hball_msp.rx_gap,
        (unsigned long)g_hball_msp.reboot_total,
        (int)g_hball_msp.heartbeat_valid,
        (int)g_hball_msp.accel_valid,
        (int)g_hball_msp.gyro_valid,
        (int)g_hball_msp.attitude_valid,
        (int)g_hball_msp.wheel_valid,
        (unsigned)g_hball_msp.status_flags
    );
    rt_kprintf(
        "[hball-mission] valid=%u epoch=%u q=%u state=%u ready=0x%04x required=0x%04x reason=%u intent=%lu/%lu chassis=%lu/%lu status_tx=%lu/%lu start=%lu/%lu ACTUATOR_TX=0\n",
        (unsigned int)g_hball_mission_arbiter.context_valid,
        (unsigned int)g_hball_mission_arbiter.epoch,
        (unsigned int)g_hball_mission_arbiter.mission_id,
        (unsigned int)g_hball_mission_arbiter.global_state,
        (unsigned int)g_hball_mission_arbiter.ready_mask,
        (unsigned int)g_hball_mission_arbiter.required_mask,
        (unsigned int)g_hball_mission_arbiter.reason,
        (unsigned long)g_hball_mission_intent_rx,
        (unsigned long)g_hball_mission_intent_invalid,
        (unsigned long)g_hball_mission_chassis_rx,
        (unsigned long)g_hball_mission_chassis_invalid,
        (unsigned long)g_hball_mission_status_tx,
        (unsigned long)g_hball_mission_status_tx_failure,
        (unsigned long)g_hball_mission_arbiter.start_accept_total,
        (unsigned long)g_hball_mission_arbiter.start_reject_total
    );
    if (g_hball_mission_intent_invalid != 0U)
    {
        rt_kprintf(
            "[hball-mission] last_invalid_intent=%02x %02x %02x %02x %02x %02x %02x %02x\n",
            g_hball_mission_last_invalid_intent[0],
            g_hball_mission_last_invalid_intent[1],
            g_hball_mission_last_invalid_intent[2],
            g_hball_mission_last_invalid_intent[3],
            g_hball_mission_last_invalid_intent[4],
            g_hball_mission_last_invalid_intent[5],
            g_hball_mission_last_invalid_intent[6],
            g_hball_mission_last_invalid_intent[7]
        );
    }
    if (g_hball_last_rx_valid)
    {
        rt_kprintf(
            "[hball-m33] last_rx id=0x%08lx ide=%u rtr=%u dlc=%u data=",
            (unsigned long)g_hball_last_rx.id,
            (unsigned int)g_hball_last_rx.is_extended,
            (unsigned int)g_hball_last_rx.is_remote,
            (unsigned int)g_hball_last_rx.dlc
        );
        for (index = 0U; index < g_hball_last_rx.dlc; ++index)
        {
            rt_kprintf(
                "%02x%s",
                (unsigned int)g_hball_last_rx.data[index],
                (index + 1U == g_hball_last_rx.dlc) ? "" : " "
            );
        }
        rt_kprintf("\n");
    }
    rt_kprintf(
        "[hball-m33] diag ret=%d ready=%d bitrate=%lu pclk=%lu cccr=%08lx psr=%08lx ecr=%08lx ir=%08lx\n",
        result,
        (int)diagnostic.ready,
        (unsigned long)diagnostic.bitrate,
        (unsigned long)diagnostic.pclk_hz,
        (unsigned long)diagnostic.cccr,
        (unsigned long)diagnostic.psr,
        (unsigned long)diagnostic.ecr,
        (unsigned long)diagnostic.ir
    );
    rt_kprintf(
        "[hball-m33] diag rxf0s=%08lx txbrp=%08lx txbto=%08lx txbcf=%08lx tx_timeout=%lu tx_fail=%lu rx_extract_fail=%lu fifo_lost=%lu fifo_full=%lu\n",
        (unsigned long)diagnostic.rxf0s,
        (unsigned long)diagnostic.txbrp,
        (unsigned long)diagnostic.txbto,
        (unsigned long)diagnostic.txbcf,
        (unsigned long)diagnostic.tx_timeout_count,
        (unsigned long)diagnostic.tx_send_fail_count,
        (unsigned long)diagnostic.rx_extract_fail_count,
        (unsigned long)diagnostic.rx_fifo0_lost_count,
        (unsigned long)diagnostic.rx_fifo0_full_count
    );
}
MSH_CMD_EXPORT(hball_status, show CAN and bounded manual-motion diagnostics);

static void hball_probe5(void)
{
#if HBALL_RS00_MOTION_TX_ENABLED
    if (hball_motion_queue_request(
            HBALL_MOTION_REQUEST_PROBE, 0.0F) != RT_EOK)
    {
        rt_kprintf("[hball-motion] probe request busy\n");
    }
#else
    (void)hball_send_probe5_once();
#endif
}
MSH_CMD_EXPORT(hball_probe5, send only motor-5 Get_ID probe without motion);

#if HBALL_RS00_MOTION_TX_ENABLED
static rt_bool_t hball_motion_confirmed(int argc, char **argv)
{
    return (argc == 2)
        && (rt_strcmp(argv[1], HBALL_RS00_CONFIRM_TOKEN) == 0);
}

static int hball_motor_prepare5(int argc, char **argv)
{
    if (!hball_motion_confirmed(argc, argv))
    {
        rt_kprintf(
            "usage: hball_motor_prepare5 %s\n", HBALL_RS00_CONFIRM_TOKEN
        );
        return -RT_EINVAL;
    }
    return hball_motion_queue_request(
        HBALL_MOTION_REQUEST_PREPARE, 0.0F
    );
}
MSH_CMD_EXPORT(
    hball_motor_prepare5,
    safely stop and prepare motor-5 CSP while remaining disabled
);

static int hball_motor_arm5(int argc, char **argv)
{
    if (!hball_motion_confirmed(argc, argv))
    {
        rt_kprintf(
            "usage: hball_motor_arm5 %s\n", HBALL_RS00_CONFIRM_TOKEN
        );
        return -RT_EINVAL;
    }
    return hball_motion_queue_request(HBALL_MOTION_REQUEST_ARM, 0.0F);
}
MSH_CMD_EXPORT(
    hball_motor_arm5,
    enable prepared motor-5 and require immediate feedback
);

static int hball_motor_step5(int argc, char **argv)
{
    char *end = RT_NULL;
    long step_mrad;

    if ((argc != 3)
        || (rt_strcmp(argv[1], HBALL_RS00_CONFIRM_TOKEN) != 0))
    {
        rt_kprintf(
            "usage: hball_motor_step5 %s <mrad -20..20>\n",
            HBALL_RS00_CONFIRM_TOKEN
        );
        return -RT_EINVAL;
    }
    step_mrad = strtol(argv[2], &end, 10);
    if ((end == argv[2]) || (*end != '\0') || (step_mrad == 0L)
        || (step_mrad < -20L) || (step_mrad > 20L))
    {
        rt_kprintf("[hball-motion] step must be nonzero -20..20 mrad\n");
        return -RT_EINVAL;
    }
    return hball_motion_queue_request(
        HBALL_MOTION_REQUEST_STEP, (float)step_mrad / 1000.0F
    );
}
MSH_CMD_EXPORT(
    hball_motor_step5,
    command one bounded manual motor-5 CSP step in mrad
);

static int hball_motor_return5(void)
{
    return hball_motion_queue_request(
        HBALL_MOTION_REQUEST_RETURN, 0.0F
    );
}
MSH_CMD_EXPORT(
    hball_motor_return5,
    return motor-5 to captured hold position then auto-stop
);

static int hball_motor_stop5(void)
{
    return hball_motion_queue_request(HBALL_MOTION_REQUEST_STOP, 0.0F);
}
MSH_CMD_EXPORT(
    hball_motor_stop5,
    always request immediate motor-5 stop without a token
);

#if HBALL_INTEGRATED_SHADOW
static int hball_q3_start_common(void)
{
    hball_sensor_snapshot_t snapshot;
    float current_pipe_rad;
    float geometric_motor_rad;
    float initial_position_m;
    const rt_uint32_t now_ms = hball_now_ms();

    if ((g_hball_motion.state != HBALL_RS00_BENCH_ARMED)
        || !hball_m33_inputs_get_snapshot(&snapshot)
        || !hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_POSITION,
            HBALL_RS00_PARAMETER_SLOT_MECH_POSITION,
            now_ms)
        || !hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_VELOCITY,
            HBALL_RS00_PARAMETER_SLOT_MECH_VELOCITY,
            now_ms))
    {
        rt_kprintf("[hball-q3] start rejected state/motor feedback\n");
        return -RT_ERROR;
    }
    initial_position_m =
        ((snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) != 0U)
        && isfinite(snapshot.ball_position_m)
        && (fabsf(snapshot.ball_position_m)
            < HBALL_BALL_COMMISSION_POSITION_LIMIT_M)
        ? snapshot.ball_position_m : 0.0F;
    hball_control_pipeline_init(
        &g_hball_ball_pipeline, initial_position_m
    );
    g_hball_ball_level_rad = HBALL_BALL_COMMISSION_LEVEL_RAD;
    if (!hball_control_pipeline_set_motor_level(
            &g_hball_ball_pipeline, g_hball_ball_level_rad))
    {
        return -RT_ERROR;
    }
    geometric_motor_rad =
        g_hball_ball_pipeline.fourbar.geometric_level_motor_angle_rad
        + HBALL_LINKAGE_MOTOR_DIRECTION_SIGN
            * (g_hball_motor.parameters.mech_position_rad
                - g_hball_ball_level_rad);
    if (!hball_fourbar_forward(
            &g_hball_ball_pipeline.fourbar,
            geometric_motor_rad,
            &current_pipe_rad))
    {
        rt_kprintf("[hball-q3] start rejected fourbar position\n");
        return -RT_ERROR;
    }
    rt_memset(&g_hball_ball_output, 0, sizeof(g_hball_ball_output));
    g_hball_ball_output.shadow_command_rad = current_pipe_rad;
    g_hball_ball_output.motor_target_rad =
        g_hball_motor.parameters.mech_position_rad;
    g_hball_ball_target_m = 0.0F;
    g_hball_ball_start_ms = 0U;
    g_hball_ball_last_step_ms = now_ms;
    g_hball_ball_sensor_invalid_since_ms = 0U;
    g_hball_ball_vision_lost = RT_FALSE;
    g_hball_ball_last_tx_ms = now_ms;
    g_hball_ball_settle_since_ms = 0U;
    g_hball_ball_phase = 0U;
    g_hball_ball_mode = 1U;
    g_hball_ball_q3_passed = RT_FALSE;
    g_hball_ball_q3_pipe_command_rad = current_pipe_rad;
    g_hball_ball_position_integral = 0.0F;
    g_hball_ball_previous_error_m = 0.0F;
    g_hball_ball_pid_static_boost_active = RT_FALSE;
    g_hball_ball_max_abs_error_m = 0.0F;
    g_hball_ball_error_violation_total = 0U;
    g_hball_ball_settle_hold = RT_FALSE;
    g_hball_ball_q3_leveling = RT_TRUE;
    g_hball_ball_q3_leveling_first_target = RT_TRUE;
    g_hball_ball_active = RT_TRUE;
    g_hball_motion.last_manual_command_ms = now_ms;
    rt_kprintf(
        "[hball-q3] pre-level motor=%ld->%ld mrad pipe=%ld mrad; then recover O\n",
        (long)(g_hball_motor.parameters.mech_position_rad * 1000.0F),
        (long)(g_hball_ball_level_rad * 1000.0F),
        (long)(current_pipe_rad * 1000.0F)
    );
    return RT_EOK;
}

static int hball_q3_start5(int argc, char **argv)
{
    if (!hball_motion_confirmed(argc, argv))
    {
        rt_kprintf("usage: hball_q3_start5 %s\n", HBALL_RS00_CONFIRM_TOKEN);
        return -RT_EINVAL;
    }
    return hball_q3_start_common();
}
MSH_CMD_EXPORT(
    hball_q3_start5,
    run real vision PID O to plus50 to minus50 mm commissioning
);

static int hball_control_pid5(void)
{
    if (g_hball_ball_active)
    {
        rt_kprintf("[hball-control] algorithm switch rejected active\n");
        return -RT_EBUSY;
    }
    g_hball_ball_use_lqi = RT_FALSE;
    rt_kprintf(
        "[hball-control] algorithm=PID kp_x1000=%ld ki_x1000=%ld kd_x1000=%ld\n",
        (long)(g_hball_ball_pid_kp * 1000.0F),
        (long)(g_hball_ball_pid_ki * 1000.0F),
        (long)(g_hball_ball_pid_kd * 1000.0F)
    );
    return RT_EOK;
}
MSH_CMD_EXPORT(hball_control_pid5, select proven PID ball controller);

static int hball_control_pid_gain5(int argc, char **argv)
{
    long kp_x1000;
    long ki_x1000;
    long kd_x1000;

    if (argc != 4)
    {
        rt_kprintf(
            "usage: hball_control_pid_gain5 <kp_x1000> <ki_x1000> <kd_x1000>\n"
        );
        return -RT_EINVAL;
    }
    if (g_hball_ball_active)
    {
        rt_kprintf("[hball-control] PID gain change rejected active\n");
        return -RT_EBUSY;
    }
    kp_x1000 = strtol(argv[1], RT_NULL, 10);
    ki_x1000 = strtol(argv[2], RT_NULL, 10);
    kd_x1000 = strtol(argv[3], RT_NULL, 10);
    if ((kp_x1000 < 300L) || (kp_x1000 > 1200L)
        || (ki_x1000 < 0L) || (ki_x1000 > 600L)
        || (kd_x1000 < 150L) || (kd_x1000 > 600L))
    {
        rt_kprintf("[hball-control] PID gains outside commissioning bounds\n");
        return -RT_EINVAL;
    }
    g_hball_ball_pid_kp = (float)kp_x1000 / 1000.0F;
    g_hball_ball_pid_ki = (float)ki_x1000 / 1000.0F;
    g_hball_ball_pid_kd = (float)kd_x1000 / 1000.0F;
    rt_kprintf(
        "[hball-control] PID gains kp_x1000=%ld ki_x1000=%ld kd_x1000=%ld\n",
        kp_x1000,
        ki_x1000,
        kd_x1000
    );
    return RT_EOK;
}
MSH_CMD_EXPORT(
    hball_control_pid_gain5,
    set bounded PID gains in x1000 units while inactive
);

static int hball_control_pid_friction5(int argc, char **argv)
{
    long boost_mrad;

    if (argc != 2)
    {
        rt_kprintf("usage: hball_control_pid_friction5 <boost_mrad 0..20>\n");
        return -RT_EINVAL;
    }
    if (g_hball_ball_active)
    {
        rt_kprintf("[hball-control] friction change rejected active\n");
        return -RT_EBUSY;
    }
    boost_mrad = strtol(argv[1], RT_NULL, 10);
    if ((boost_mrad < 0L) || (boost_mrad > 20L))
    {
        rt_kprintf("[hball-control] friction boost outside 0..20 mrad\n");
        return -RT_EINVAL;
    }
    g_hball_ball_pid_static_boost_rad = (float)boost_mrad / 1000.0F;
    rt_kprintf(
        "[hball-control] PID static friction boost_mrad=%ld\n",
        boost_mrad
    );
    return RT_EOK;
}
MSH_CMD_EXPORT(
    hball_control_pid_friction5,
    set low-speed static friction compensation in mrad while inactive
);

static int hball_control_lqi5(void)
{
    if (g_hball_ball_active)
    {
        rt_kprintf("[hball-control] algorithm switch rejected active\n");
        return -RT_EBUSY;
    }
    g_hball_ball_use_lqi = RT_TRUE;
    rt_kprintf(
        "[hball-control] algorithm=LQI kx=1.576194 ki=1.0 kv=0.713641\n"
    );
    return RT_EOK;
}
MSH_CMD_EXPORT(hball_control_lqi5, select MATLAB designed LQI controller);

static int hball_hold_start_common(
    rt_uint8_t mode, float target_m, const char *name
)
{
    hball_sensor_snapshot_t snapshot;
    const rt_uint32_t now_ms = hball_now_ms();

    if ((g_hball_motion.state != HBALL_RS00_BENCH_ARMED)
        || !hball_m33_inputs_get_snapshot(&snapshot)
        || ((snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) == 0U)
        || !hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_POSITION,
            HBALL_RS00_PARAMETER_SLOT_MECH_POSITION,
            now_ms)
        || !hball_motion_parameter_fresh(
            HBALL_RS00_PARAMETER_VALID_MECH_VELOCITY,
            HBALL_RS00_PARAMETER_SLOT_MECH_VELOCITY,
            now_ms)
        || (fabsf(snapshot.ball_position_m)
            >= HBALL_BALL_COMMISSION_POSITION_LIMIT_M)
        || (fabsf(target_m) > 0.080F))
    {
        rt_kprintf("[hball-hold] start rejected state/input/position\n");
        return -RT_ERROR;
    }
    hball_control_pipeline_init(
        &g_hball_ball_pipeline, snapshot.ball_position_m
    );
    g_hball_ball_level_rad = HBALL_BALL_COMMISSION_LEVEL_RAD;
    if (!hball_control_pipeline_set_motor_level(
            &g_hball_ball_pipeline, g_hball_ball_level_rad))
    {
        return -RT_ERROR;
    }
    rt_memset(&g_hball_ball_output, 0, sizeof(g_hball_ball_output));
    g_hball_ball_target_m = target_m;
    g_hball_ball_start_ms = now_ms;
    g_hball_ball_last_step_ms = now_ms;
    g_hball_ball_sensor_invalid_since_ms = 0U;
    g_hball_ball_vision_lost = RT_FALSE;
    g_hball_ball_last_tx_ms = now_ms;
    g_hball_ball_settle_since_ms = 0U;
    g_hball_ball_phase = (mode == 2U) ? 5U : 6U;
    g_hball_ball_mode = mode;
    g_hball_ball_q3_passed = RT_FALSE;
    g_hball_ball_position_integral = 0.0F;
    g_hball_ball_previous_error_m = 0.0F;
    g_hball_ball_pid_static_boost_active = RT_FALSE;
    g_hball_ball_max_abs_error_m = 0.0F;
    g_hball_ball_error_violation_total = 0U;
    g_hball_ball_settle_hold = RT_FALSE;
    g_hball_ball_active = RT_TRUE;
    g_hball_motion.last_manual_command_ms = now_ms;
    rt_kprintf(
        "[hball-hold] start mode=%s x_mm=%ld target_mm=%ld timeout_ms=%u\n",
        name,
        (long)(snapshot.ball_position_m * 1000.0F),
        (long)(target_m * 1000.0F),
        (unsigned int)HBALL_BALL_HOLD_TIMEOUT_MS
    );
    return RT_EOK;
}

static int hball_hold_center5(int argc, char **argv)
{
    if (!hball_motion_confirmed(argc, argv))
    {
        rt_kprintf(
            "usage: hball_hold_center5 %s\n", HBALL_RS00_CONFIRM_TOKEN
        );
        return -RT_EINVAL;
    }
    return hball_hold_start_common(2U, 0.0F, "center");
}
MSH_CMD_EXPORT(
    hball_hold_center5,
    manually invoke the formal Q4 and Q5 center controller
);

static int hball_hold_latch5(int argc, char **argv)
{
    hball_sensor_snapshot_t snapshot;

    if (!hball_motion_confirmed(argc, argv))
    {
        rt_kprintf(
            "usage: hball_hold_latch5 %s\n", HBALL_RS00_CONFIRM_TOKEN
        );
        return -RT_EINVAL;
    }
    if (!hball_m33_inputs_get_snapshot(&snapshot)
        || ((snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) == 0U))
    {
        return -RT_ERROR;
    }
    return hball_hold_start_common(
        3U, snapshot.ball_position_m, "latched"
    );
}
MSH_CMD_EXPORT(
    hball_hold_latch5,
    manually invoke the formal Q6 latched-position controller
);

static int hball_control_level5(void)
{
    if (!g_hball_ball_active)
    {
        rt_kprintf("[hball-control] level rejected inactive\n");
        return -RT_ERROR;
    }
    g_hball_ball_phase = 4U;
    g_hball_ball_settle_since_ms = 0U;
    rt_kprintf("[hball-control] returning level before stop\n");
    return RT_EOK;
}
MSH_CMD_EXPORT(
    hball_control_level5,
    return pipe to calibrated level then stop
);

static void hball_q3_status5(void)
{
    hball_sensor_snapshot_t snapshot;

    if (!hball_m33_inputs_get_snapshot(&snapshot))
    {
        rt_kprintf("[hball-q3] snapshot unavailable\n");
        return;
    }
    rt_kprintf(
        "[hball-control] algo=%s active=%d mode=%u phase=%u leveling=%d passed=%d x_mm=%ld target_mm=%ld estimate_mm=%ld velocity_mm_s=%ld disturbance_mm_s2=%ld pipe_mrad=%ld motor_target_mrad=%ld max_error_mm=%ld violations=%lu tx=%lu\n",
        (g_hball_ball_mode == 1U)
            ? (g_hball_ball_use_lqi ? "LQI" : "PID")
            : "OOSM-LQI",
        (int)g_hball_ball_active,
        (unsigned int)g_hball_ball_mode,
        (unsigned int)g_hball_ball_phase,
        (int)g_hball_ball_q3_leveling,
        (int)g_hball_ball_q3_passed,
        (long)(snapshot.ball_position_m * 1000.0F),
        (long)(g_hball_ball_target_m * 1000.0F),
        (long)(g_hball_ball_output.estimated_position_m * 1000.0F),
        (long)(g_hball_ball_output.estimated_velocity_mps * 1000.0F),
        (long)(g_hball_ball_output.estimated_disturbance_mps2 * 1000.0F),
        (long)(g_hball_ball_output.shadow_command_rad * 1000.0F),
        (long)(g_hball_ball_output.motor_target_rad * 1000.0F),
        (long)(g_hball_ball_max_abs_error_m * 1000.0F),
        (unsigned long)g_hball_ball_error_violation_total,
        (unsigned long)g_hball_ball_tx_total
    );
}
MSH_CMD_EXPORT(hball_q3_status5, show real Q3 commissioning status);
#endif

static void hball_motor_status5(void)
{
    rt_kprintf(
        "[hball-motion] state=%s reason=%u initial_mrad=%ld target_mrad=%ld mech_pos_mrad=%ld mech_vel_mrad_s=%ld feedback_age_ms=%lu feedback_fault=0x%02x run_mode=%u vbus_mv=%ld tx=%lu readback_tx=%lu reject=%lu auto_stop=%lu\n",
        hball_rs00_bench_state_name(g_hball_motion.state),
        (unsigned int)g_hball_motion.stop_reason,
        (long)(g_hball_motion.initial_position_rad * 1000.0F),
        (long)(g_hball_motion.target_position_rad * 1000.0F),
        (long)(g_hball_motor.parameters.mech_position_rad * 1000.0F),
        (long)(g_hball_motor.parameters.mech_velocity_rad_s * 1000.0F),
        (unsigned long)(hball_now_ms() - g_hball_motor.last_feedback_ms),
        (unsigned int)g_hball_motor.feedback.fault_summary,
        (unsigned int)g_hball_motor.parameters.run_mode,
        (long)(g_hball_motor.parameters.vbus_v * 1000.0F),
        (unsigned long)g_hball_motion_tx_total,
        (unsigned long)g_hball_motion_readback_tx_total,
        (unsigned long)g_hball_motion_reject_total,
        (unsigned long)g_hball_motion_auto_stop_total
    );
}
MSH_CMD_EXPORT(
    hball_motor_status5,
    show bounded manual motor-5 session state and safety data
);

static void hball_motor_trace5(void)
{
    rt_uint16_t count;

    if (g_hball_step_trace_active)
    {
        rt_kprintf("[hball-trace] capture still active\n");
        return;
    }
    g_hball_motion_return_settle_since_ms = 0U;
    count = g_hball_step_trace_count;
    rt_kprintf(
        "[hball-trace] count=%u columns=t_ms,target_mrad,pos_mrad,vel_mrad_s,fault\n",
        (unsigned int)count
    );
    for (rt_uint16_t index = 0U; index < count; ++index)
    {
        const hball_step_trace_sample_t *sample =
            &g_hball_step_trace[index];

        rt_kprintf(
            "%lu,%ld,%ld,%ld,%u\n",
            (unsigned long)sample->elapsed_ms,
            (long)(sample->target_rad * 1000.0F),
            (long)(sample->position_rad * 1000.0F),
            (long)(sample->velocity_rad_s * 1000.0F),
            (unsigned int)sample->fault
        );
    }
}
MSH_CMD_EXPORT(
    hball_motor_trace5,
    dump the last bounded RS00 step response trace
);
#endif

static int hball_bench_start(void)
{
    hball_motor_monitor_init(&g_hball_motor, HBALL_RS00_MOTOR_ID);
    hball_msp_monitor_init(&g_hball_msp);
    hball_mission_arbiter_init(&g_hball_mission_arbiter);
    hball_m33_q456_init(&g_hball_q456_runtime);
    rt_memset(&g_hball_mission_chassis, 0, sizeof(g_hball_mission_chassis));
    hball_rate_meter_init(&g_hball_can_rx_rate);
#if HBALL_RS00_MOTION_TX_ENABLED
    hball_rs00_bench_init(&g_hball_motion);
    g_hball_motion_mutex = rt_mutex_create("hbmot", RT_IPC_FLAG_PRIO);
    if (g_hball_motion_mutex == RT_NULL)
    {
        rt_kprintf("[hball-motion] failed to create command mutex\n");
        return -RT_ENOMEM;
    }
    rt_kprintf(
        "[hball-m33] SAFETY boot-disarmed: formal Q4-Q6 CSP speed=3.0 rad/s current=0.8 A; manual commands still require token\n"
    );
#else
    rt_kprintf(
        "[hball-m33] SAFETY read-only: RS00 parameter reads=%u; no enable, zero, mode, position, speed, current or torque TX\n",
        (unsigned int)HBALL_RS00_READBACK_TX_ENABLED
    );
#endif
    hball_init();
    if (!g_hball_can_ready)
    {
        return -RT_ERROR;
    }
#if HBALL_RS00_MOTION_TX_ENABLED
    /* RS00 can retain enable state across an M33-only reset. */
    hball_motion_stop_now(
        HBALL_RS00_BENCH_STOP_MANUAL, hball_now_ms(), RT_FALSE
    );
#endif

    g_hball_worker = rt_thread_create(
        "hball_can",
        hball_worker_entry,
        RT_NULL,
        4096U,
        12U,
        10U
    );
    if (g_hball_worker == RT_NULL)
    {
        return -RT_ERROR;
    }
    rt_thread_startup(g_hball_worker);
    return RT_EOK;
}
INIT_APP_EXPORT(hball_bench_start);
