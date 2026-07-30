#include "hball_can_port.h"

#include "hball_can_protocol.h"
#include "ti_msp_dl_config.h"
#include "wit.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* Automated builds and tests must never create an actuator command frame. */
#define HBALL_CAN_MOTOR_COMMAND_TX_ENABLED 0U

#define HBALL_CAN_TX_BUFFER_INDEX 0U
#define HBALL_CAN_IMU_FRESH_MS 100U
#define HBALL_CAN_ENCODER_COUNTS_PER_REVOLUTION 400U
/* Set only after measuring the installed wheel; zero keeps SI wheel data safe. */
#define HBALL_CAN_WHEEL_CIRCUMFERENCE_MM 0U

extern int32_t encoderA_cnt;
extern int32_t encoderB_cnt;

volatile hball_can_port_stats_t g_hball_can_stats;

static uint16_t g_hball_sequences[HBALL_CAN_STREAM_COUNT];
static uint32_t g_hball_last_wit_count;
static uint32_t g_hball_last_wit_ms;
static bool g_hball_wit_seen;

static void hball_can_snapshot_inputs(
    uint32_t now_ms, hball_can_inputs_t *inputs
)
{
    uint32_t wit_count;
    int32_t left_milli_mps;
    int32_t right_milli_mps;

    memset(inputs, 0, sizeof(*inputs));
    inputs->uptime_ms = now_ms;

    /* No hardware E-stop input is mapped yet, so telemetry remains inhibited. */
    inputs->status_flags = HBALL_MSP_STATUS_ESTOP_ACTIVE;
    wit_count = wit_valid_frame_count;
    if (wit_count != g_hball_last_wit_count)
    {
        g_hball_last_wit_count = wit_count;
        g_hball_last_wit_ms = now_ms;
        g_hball_wit_seen = true;
    }
    if (g_hball_wit_seen
        && (wit_angle_frame_count != 0U)
        && ((uint32_t)(now_ms - g_hball_last_wit_ms) <= HBALL_CAN_IMU_FRESH_MS))
    {
        inputs->status_flags |= HBALL_MSP_STATUS_IMU_VALID;
    }

    inputs->accel_milli_mps2[0] = hball_can_mg_to_milli_mps2(wit_data.ax);
    inputs->accel_milli_mps2[1] = hball_can_mg_to_milli_mps2(wit_data.ay);
    inputs->accel_milli_mps2[2] = hball_can_mg_to_milli_mps2(wit_data.az);
    inputs->gyro_milli_rad_s[0] = hball_can_dps_to_milli_rad_s(wit_data.gx);
    inputs->gyro_milli_rad_s[1] = hball_can_dps_to_milli_rad_s(wit_data.gy);
    inputs->gyro_milli_rad_s[2] = hball_can_dps_to_milli_rad_s(wit_data.gz);
    inputs->attitude_milli_rad[0] = hball_can_deg_to_milli_rad(wit_data.roll);
    inputs->attitude_milli_rad[1] = hball_can_deg_to_milli_rad(wit_data.pitch);
    inputs->attitude_milli_rad[2] = hball_can_deg_to_milli_rad(wit_data.yaw);

    left_milli_mps = hball_can_counts_per_20ms_to_milli_mps(
        encoderA_cnt,
        HBALL_CAN_WHEEL_CIRCUMFERENCE_MM,
        HBALL_CAN_ENCODER_COUNTS_PER_REVOLUTION
    );
    right_milli_mps = hball_can_counts_per_20ms_to_milli_mps(
        encoderB_cnt,
        HBALL_CAN_WHEEL_CIRCUMFERENCE_MM,
        HBALL_CAN_ENCODER_COUNTS_PER_REVOLUTION
    );
    inputs->wheel_milli_mps[0] = (int16_t)left_milli_mps;
    inputs->wheel_milli_mps[1] = (int16_t)right_milli_mps;
    inputs->body_milli_mps = (int16_t)((left_milli_mps + right_milli_mps) / 2);
}

static void hball_can_frame_to_tx_element(
    const hball_can_frame_t *frame, DL_MCAN_TxBufElement *element
)
{
    memset(element, 0, sizeof(*element));
    element->id = frame->id << 18U;
    element->rtr = frame->is_remote;
    element->xtd = frame->is_extended;
    element->dlc = frame->dlc;
    element->brs = frame->brs;
    element->fdf = frame->fdf;
    memcpy(element->data, frame->data, sizeof(frame->data));
}

static void hball_can_refresh_error_status(void)
{
    DL_MCAN_ErrCntStatus errors;
    DL_MCAN_ProtocolStatus protocol;

    DL_MCAN_getErrCounters(MCAN0_INST, &errors);
    DL_MCAN_getProtocolStatus(MCAN0_INST, &protocol);
    g_hball_can_stats.tx_error_count = (uint16_t)errors.transErrLogCnt;
    g_hball_can_stats.rx_error_count = (uint16_t)errors.recErrCnt;
    g_hball_can_stats.bus_off = (uint8_t)protocol.busOffStatus;
    g_hball_can_stats.error_passive = (uint8_t)protocol.errPassive;
    g_hball_can_stats.error_warning = (uint8_t)protocol.warningStatus;
}

void hball_can_port_init(void)
{
    memset((void *)&g_hball_can_stats, 0, sizeof(g_hball_can_stats));
    memset(g_hball_sequences, 0, sizeof(g_hball_sequences));
    g_hball_last_wit_count = wit_valid_frame_count;
    g_hball_last_wit_ms = 0U;
    g_hball_wit_seen = false;

    if (DL_MCAN_getOpMode(MCAN0_INST) != DL_MCAN_OPERATION_MODE_NORMAL)
    {
        return;
    }
    hball_can_refresh_error_status();
    NVIC_ClearPendingIRQ(MCAN0_INST_INT_IRQN);
    NVIC_SetPriority(MCAN0_INST_INT_IRQN, 2U);
    NVIC_EnableIRQ(MCAN0_INST_INT_IRQN);
    g_hball_can_stats.initialized = 1U;
}

void hball_can_port_tick_1ms(uint32_t now_ms)
{
    hball_can_stream_t stream;
    hball_can_inputs_t inputs;
    hball_can_frame_t frame;
    DL_MCAN_TxBufElement tx_element;

    if ((g_hball_can_stats.initialized == 0U)
        || !hball_can_stream_due(now_ms, &stream))
    {
        return;
    }
    if ((DL_MCAN_getTxBufReqPend(MCAN0_INST)
            & (UINT32_C(1) << HBALL_CAN_TX_BUFFER_INDEX)) != 0U)
    {
        g_hball_can_stats.tx_busy++;
        return;
    }

    hball_can_snapshot_inputs(now_ms, &inputs);
    if (!hball_can_encode_frame(
            stream, g_hball_sequences[stream], &inputs, &frame))
    {
        g_hball_can_stats.tx_failed++;
        return;
    }
    hball_can_frame_to_tx_element(&frame, &tx_element);
    DL_MCAN_writeMsgRam(
        MCAN0_INST,
        DL_MCAN_MEM_TYPE_BUF,
        HBALL_CAN_TX_BUFFER_INDEX,
        &tx_element
    );
    if (DL_MCAN_TXBufAddReq(MCAN0_INST, HBALL_CAN_TX_BUFFER_INDEX) < 0)
    {
        g_hball_can_stats.tx_failed++;
        return;
    }
    g_hball_sequences[stream]++;
    g_hball_can_stats.tx_queued++;
}

static void hball_can_record_rx(const DL_MCAN_RxBufElement *message)
{
    uint32_t id;

    if (message->xtd != 0U)
    {
        id = message->id;
        g_hball_can_stats.rx_extended++;
    }
    else
    {
        id = (message->id & UINT32_C(0x1ffc0000)) >> 18U;
        g_hball_can_stats.rx_standard++;
    }

    g_hball_can_stats.rx_total++;
    g_hball_can_stats.last_rx_id = id;
    g_hball_can_stats.last_rx_extended = (uint8_t)message->xtd;
    g_hball_can_stats.last_rx_dlc = (uint8_t)message->dlc;
    if (message->rtr != 0U)
    {
        g_hball_can_stats.rx_remote++;
    }
    if ((message->fdf != 0U) || (message->brs != 0U))
    {
        g_hball_can_stats.rx_fd_rejected++;
    }
    memcpy(
        (void *)g_hball_can_stats.last_rx_data,
        message->data,
        sizeof(g_hball_can_stats.last_rx_data)
    );
}

static void hball_can_drain_fifo0(void)
{
    DL_MCAN_RxBufElement message;
    DL_MCAN_RxFIFOStatus fifo_status;

    memset(&fifo_status, 0, sizeof(fifo_status));
    fifo_status.num = DL_MCAN_RX_FIFO_NUM_0;
    DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifo_status);
    while (fifo_status.fillLvl != 0U)
    {
        DL_MCAN_readMsgRam(
            MCAN0_INST,
            DL_MCAN_MEM_TYPE_FIFO,
            0U,
            fifo_status.num,
            &message
        );
        DL_MCAN_writeRxFIFOAck(
            MCAN0_INST, fifo_status.num, fifo_status.getIdx);
        hball_can_record_rx(&message);
        DL_MCAN_getRxFIFOStatus(MCAN0_INST, &fifo_status);
    }
}

void MCAN0_INST_IRQHandler(void)
{
    uint32_t interrupt_status;

    if (DL_MCAN_getPendingInterrupt(MCAN0_INST) != DL_MCAN_IIDX_LINE1)
    {
        return;
    }
    interrupt_status = DL_MCAN_getIntrStatus(MCAN0_INST);
    DL_MCAN_clearIntrStatus(
        MCAN0_INST, interrupt_status, DL_MCAN_INTR_SRC_MCAN_LINE_1);
    g_hball_can_stats.last_irq_status = interrupt_status;

    if ((interrupt_status & (MCAN_IR_RF0N_MASK | MCAN_IR_RF0F_MASK)) != 0U)
    {
        hball_can_drain_fifo0();
    }
    if ((interrupt_status & MCAN_IR_TC_MASK) != 0U)
    {
        g_hball_can_stats.tx_confirmed++;
    }
    if ((interrupt_status & MCAN_IR_RF0F_MASK) != 0U)
    {
        g_hball_can_stats.rx_fifo_full++;
    }
    if ((interrupt_status & MCAN_IR_RF0L_MASK) != 0U)
    {
        g_hball_can_stats.rx_fifo_lost++;
    }
    if ((interrupt_status & MCAN_IR_BO_MASK) != 0U)
    {
        g_hball_can_stats.bus_off_events++;
    }
    if ((interrupt_status & (MCAN_IR_PEA_MASK | MCAN_IR_PED_MASK)) != 0U)
    {
        g_hball_can_stats.protocol_error_events++;
    }
    if ((interrupt_status & MCAN_IR_MRAF_MASK) != 0U)
    {
        g_hball_can_stats.message_ram_errors++;
    }
    hball_can_refresh_error_status();
}
