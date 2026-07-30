#include "hball_can.h"
#include "hball_rate_meter.h"
#include "hball_diagnostics_config.h"
#include "hball_rs00_control.h"
#if HBALL_INTEGRATED_SHADOW
#include "hball_m33_inputs.h"
#endif

#include "drv_can.h"
#include <finsh.h>
#include <math.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <stdlib.h>

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
#define HBALL_BENCH_VERSION "0.5.0-m33-manual-small-step"
#else
#define HBALL_BENCH_VERSION "0.3.0-m33-can-readback"
#endif
#define HBALL_BENCH_PERIOD_MS 1U
#define HBALL_BENCH_LOG_PERIOD_MS 1000U
#define HBALL_BENCH_AUTO_PROBE_DELAY_MS 1000U
#define HBALL_BENCH_RX_BUDGET 32U
#define HBALL_RS00_READBACK_PERIOD_MS 20U
#define HBALL_RS00_READBACK_TIMEOUT_MS 10U
#define HBALL_RS00_MOTION_PARAMETER_FRESH_MS 250U
#define HBALL_RS00_MOTION_PREPARE_TIMEOUT_MS 250U
#define HBALL_RS00_MOTION_RETURN_TIMEOUT_MS 500U
#define HBALL_RS00_MOTION_VBUS_MIN_V 18.0F
#define HBALL_RS00_MOTION_VBUS_MAX_V 26.0F
#define HBALL_RS00_MOTION_STATIONARY_RAD_S 0.2F
#define HBALL_RS00_MOTION_RETURN_TOLERANCE_RAD 0.005F
#define HBALL_RS00_MOTION_RETURN_VELOCITY_RAD_S 0.05F
#define HBALL_RS00_MOTION_READBACK_PERIOD_MS 10U
#define HBALL_RS00_MOTION_READBACK_FRESH_MS 100U
#define HBALL_RS00_PARAMETER_SLOT_MECH_POSITION 1U
#define HBALL_RS00_PARAMETER_SLOT_MECH_VELOCITY 3U
#define HBALL_RS00_CONFIRM_TOKEN "CONFIRM_NO_LOAD"

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
static rt_uint32_t g_hball_motion_readback_last_ms = 0U;
static rt_uint32_t g_hball_motion_readback_tx_total = 0U;
static rt_uint8_t g_hball_motion_readback_next = 0U;
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

    if ((frame == RT_NULL) || !g_hball_can_ready
        || (frame->is_extended == 0U) || (frame->is_remote != 0U)
        || (frame->dlc != 8U))
    {
        return -RT_ERROR;
    }
    rt_memset(&message, 0, sizeof(message));
    message.id = frame->id;
    message.ide = RT_CAN_EXTID;
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
#if HBALL_INTEGRATED_SHADOW
                (void)hball_m33_inputs_publish_motor_parameters(
                    &g_hball_motor.parameters
                );
#endif
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
    rt_kprintf(
        "[hball-motion] tx %s id=0x%08lx ret=%d\n",
        label,
        (unsigned long)((frame != RT_NULL) ? frame->id : 0U),
        result
    );
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
        "[hball-motion] PREPARING hold_mrad=%ld speed_mrad_s=%ld current_ma=%ld vbus_mv=%ld\n",
        (long)(g_hball_motion.initial_position_rad * 1000.0F),
        (long)(HBALL_RS00_BENCH_SPEED_LIMIT_RAD_S * 1000.0F),
        (long)(HBALL_RS00_BENCH_CURRENT_LIMIT_A * 1000.0F),
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
    if (g_hball_motion_prepare_stage >= 6U)
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

    if (!hball_motor_monitor_feedback_fresh(
            &g_hball_motor,
            now_ms,
            HBALL_RS00_BENCH_FEEDBACK_TIMEOUT_MS)
        || (g_hball_motor.feedback.fault_summary != 0U))
    {
        hball_motion_reject("step", "feedback stale or faulted");
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
    if ((g_hball_motion.state == HBALL_RS00_BENCH_ARMED)
        || (g_hball_motion.state == HBALL_RS00_BENCH_SMALL_STEP)
        || (g_hball_motion.state == HBALL_RS00_BENCH_RETURNING))
    {
        if (!hball_motor_monitor_feedback_fresh(
                &g_hball_motor,
                now_ms,
                HBALL_RS00_BENCH_FEEDBACK_TIMEOUT_MS)
            && !hball_motion_parameter_fresh(
                HBALL_RS00_PARAMETER_VALID_MECH_POSITION,
                HBALL_RS00_PARAMETER_SLOT_MECH_POSITION,
                now_ms))
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
        hball_motion_stop_now(
            HBALL_RS00_BENCH_STOP_MANUAL, now_ms, RT_TRUE
        );
        return;
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
#if HBALL_BENCH_AUTO_PROBE5 || HBALL_PERIODIC_DIAGNOSTICS \
    || HBALL_RS00_READBACK_TX_ENABLED || HBALL_RS00_MOTION_TX_ENABLED
        rt_uint32_t now_ms;
#endif

        hball_poll_can();
#if HBALL_BENCH_AUTO_PROBE5 || HBALL_PERIODIC_DIAGNOSTICS \
    || HBALL_RS00_READBACK_TX_ENABLED || HBALL_RS00_MOTION_TX_ENABLED
        now_ms = hball_now_ms();
#endif
#if HBALL_RS00_MOTION_TX_ENABLED
        hball_motion_tick(now_ms);
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
#endif

static int hball_bench_start(void)
{
    hball_motor_monitor_init(&g_hball_motor, HBALL_RS00_MOTOR_ID);
    hball_msp_monitor_init(&g_hball_msp);
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
        "[hball-m33] SAFETY manual motion compiled but boot-disarmed: token required; CSP speed=0.5 rad/s current=0.8 A step<=20 mrad envelope=+-50 mrad; ACTUATOR_TX=0\n"
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
