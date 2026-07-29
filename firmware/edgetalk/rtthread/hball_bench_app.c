#include "hball_can.h"
#include "hball_rate_meter.h"
#if HBALL_INTEGRATED_SHADOW
#include "hball_m33_inputs.h"
#endif

#include "drv_can.h"
#include <finsh.h>
#include <rtdevice.h>
#include <rtthread.h>

#ifndef HBALL_BENCH_AUTO_PROBE5
#define HBALL_BENCH_AUTO_PROBE5 0
#endif

#define HBALL_BENCH_VERSION "0.2.0-m33-can-only"
#define HBALL_BENCH_PERIOD_MS 1U
#define HBALL_BENCH_LOG_PERIOD_MS 1000U
#define HBALL_BENCH_AUTO_PROBE_DELAY_MS 1000U
#define HBALL_BENCH_RX_BUDGET 32U

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

static void hball_poll_can(void)
{
    struct rt_can_msg message;
    hball_can_frame_t frame;
    rt_uint8_t drained = 0U;

    while ((drained < HBALL_BENCH_RX_BUDGET)
        && (ifx_can_direct_recv(&message) == (rt_ssize_t)sizeof(message)))
    {
        drained++;
        hball_copy_from_rt_can(&frame, &message);
        g_hball_last_rx = frame;
        g_hball_last_rx_valid = RT_TRUE;
        g_hball_raw_rx_total++;
        hball_rate_meter_accept(&g_hball_can_rx_rate, hball_now_ms());
        if (frame.is_extended != 0U)
        {
            const hball_can_event_t event = hball_motor_monitor_accept(
                &g_hball_motor, &frame, hball_now_ms()
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
#if HBALL_INTEGRATED_SHADOW
                (void)hball_m33_inputs_publish_motor(
                    &g_hball_motor.feedback, hball_now_ms()
                );
#endif
            }
        }
        else
        {
            const hball_msp_event_t event = hball_msp_monitor_accept(
                &g_hball_msp, &frame, hball_now_ms()
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

static rt_err_t hball_send_probe5_once(void)
{
    hball_can_frame_t frame;
    struct rt_can_msg message;
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

    rt_memset(&message, 0, sizeof(message));
    message.id = frame.id;
    message.ide = RT_CAN_EXTID;
    message.rtr = RT_CAN_DTR;
    message.len = frame.dlc;
    message.hdr_index = -1;
    rt_memcpy(message.data, frame.data, frame.dlc);

    result = ifx_can_direct_send(&message);
    g_hball_tx_total++;
    if (result == RT_EOK)
    {
        g_hball_tx_success++;
    }
    else
    {
        g_hball_tx_failure++;
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

static void hball_worker_entry(void *parameter)
{
    rt_uint32_t last_log_ms = hball_now_ms();
    rt_uint32_t boot_ms = last_log_ms;
    rt_bool_t auto_probe_done = RT_FALSE;

    RT_UNUSED(parameter);
    while (1)
    {
        rt_uint32_t now_ms;

        hball_poll_can();
        now_ms = hball_now_ms();
#if HBALL_BENCH_AUTO_PROBE5
        if (!auto_probe_done
            && ((rt_uint32_t)(now_ms - boot_ms) >= HBALL_BENCH_AUTO_PROBE_DELAY_MS))
        {
            auto_probe_done = RT_TRUE;
            (void)hball_send_probe5_once();
        }
#else
        RT_UNUSED(boot_ms);
        RT_UNUSED(auto_probe_done);
#endif
        if ((rt_uint32_t)(now_ms - last_log_ms) >= HBALL_BENCH_LOG_PERIOD_MS)
        {
            last_log_ms = now_ms;
            rt_kprintf(
                "[hball-m33] alive can_rx=%lu tx_ok=%lu tx_fail=%lu MOTOR_COMMAND_TX=0\n",
                (unsigned long)g_hball_raw_rx_total,
                (unsigned long)g_hball_tx_success,
                (unsigned long)g_hball_tx_failure
            );
        }
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
MSH_CMD_EXPORT(hball_init, initialize read-only H-ball CAN monitor);

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
        "[hball-m33] status version=%s read_only=1 can_ready=%d tx=%lu/%lu/%lu raw_rx=%lu can_rate_x10=%lu\n",
        HBALL_BENCH_VERSION,
        (int)g_hball_can_ready,
        (unsigned long)g_hball_tx_total,
        (unsigned long)g_hball_tx_success,
        (unsigned long)g_hball_tx_failure,
        (unsigned long)g_hball_raw_rx_total,
        (unsigned long)can_rate_x10
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
        "[hball-m33] msp_rx=%lu invalid=%lu ignored=%lu hb=%d accel=%d gyro=%d attitude=%d wheel=%d status=0x%04x\n",
        (unsigned long)g_hball_msp.rx_total,
        (unsigned long)g_hball_msp.rx_invalid,
        (unsigned long)g_hball_msp.rx_ignored,
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
MSH_CMD_EXPORT(hball_status, show read-only CAN diagnostics);

static void hball_probe5(void)
{
    (void)hball_send_probe5_once();
}
MSH_CMD_EXPORT(hball_probe5, send only motor-5 Get_ID probe without motion);

static int hball_bench_start(void)
{
    hball_motor_monitor_init(&g_hball_motor, HBALL_RS00_MOTOR_ID);
    hball_msp_monitor_init(&g_hball_msp);
    hball_rate_meter_init(&g_hball_can_rx_rate);
    rt_kprintf(
        "[hball-m33] SAFETY CAN-only: no enable, zero, position, speed or torque TX\n"
    );
    hball_init();
    if (!g_hball_can_ready)
    {
        return -RT_ERROR;
    }

    g_hball_worker = rt_thread_create(
        "hball_can",
        hball_worker_entry,
        RT_NULL,
        3072U,
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
