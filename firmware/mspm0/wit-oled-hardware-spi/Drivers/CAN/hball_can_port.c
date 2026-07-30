#include "hball_can_port.h"

#include "hball_can_protocol.h"
#include "hball_can_recovery.h"
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
static uint32_t g_hball_last_wit_accel_count;
static uint32_t g_hball_last_wit_gyro_count;
static uint32_t g_hball_last_wit_attitude_count;
static uint32_t g_hball_last_wit_accel_ms;
static uint32_t g_hball_last_wit_gyro_ms;
static uint32_t g_hball_last_wit_attitude_ms;
static bool g_hball_wit_accel_seen;
static bool g_hball_wit_gyro_seen;
static bool g_hball_wit_attitude_seen;
static hball_can_recovery_t g_hball_can_recovery;

static void hball_can_update_wit_freshness(uint32_t now_ms)
{
    const uint32_t accel_count = wit_accel_frame_count;
    const uint32_t gyro_count = wit_gyro_frame_count;
    const uint32_t attitude_count = wit_angle_frame_count;

    if (accel_count != g_hball_last_wit_accel_count)
    {
        g_hball_last_wit_accel_count = accel_count;
        g_hball_last_wit_accel_ms = now_ms;
        g_hball_wit_accel_seen = true;
    }
    if (gyro_count != g_hball_last_wit_gyro_count)
    {
        g_hball_last_wit_gyro_count = gyro_count;
        g_hball_last_wit_gyro_ms = now_ms;
        g_hball_wit_gyro_seen = true;
    }
    if (attitude_count != g_hball_last_wit_attitude_count)
    {
        g_hball_last_wit_attitude_count = attitude_count;
        g_hball_last_wit_attitude_ms = now_ms;
        g_hball_wit_attitude_seen = true;
    }
}

static void hball_can_snapshot_wit(hball_can_inputs_t *inputs)
{
    uint32_t before;
    uint32_t after;
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;
    float angle_x;
    float angle_y;
    float angle_z;

    do
    {
        before = wit_accel_frame_count;
        raw_x = wit_data.ax;
        raw_y = wit_data.ay;
        raw_z = wit_data.az;
        after = wit_accel_frame_count;
    } while (before != after);
    inputs->accel_source_sequence = (uint16_t)after;
    inputs->accel_milli_mps2[0] = hball_can_mg_to_milli_mps2(raw_x);
    inputs->accel_milli_mps2[1] = hball_can_mg_to_milli_mps2(raw_y);
    inputs->accel_milli_mps2[2] = hball_can_mg_to_milli_mps2(raw_z);

    do
    {
        before = wit_gyro_frame_count;
        raw_x = wit_data.gx;
        raw_y = wit_data.gy;
        raw_z = wit_data.gz;
        after = wit_gyro_frame_count;
    } while (before != after);
    inputs->gyro_source_sequence = (uint16_t)after;
    inputs->gyro_milli_rad_s[0] = hball_can_dps_to_milli_rad_s(raw_x);
    inputs->gyro_milli_rad_s[1] = hball_can_dps_to_milli_rad_s(raw_y);
    inputs->gyro_milli_rad_s[2] = hball_can_dps_to_milli_rad_s(raw_z);

    do
    {
        before = wit_angle_frame_count;
        angle_x = wit_data.roll;
        angle_y = wit_data.pitch;
        angle_z = wit_data.yaw;
        after = wit_angle_frame_count;
    } while (before != after);
    inputs->attitude_source_sequence = (uint16_t)after;
    inputs->attitude_milli_rad[0] = hball_can_deg_to_milli_rad(angle_x);
    inputs->attitude_milli_rad[1] = hball_can_deg_to_milli_rad(angle_y);
    inputs->attitude_milli_rad[2] = hball_can_deg_to_milli_rad(angle_z);
}

static void hball_can_snapshot_inputs(
    uint32_t now_ms, hball_can_inputs_t *inputs
)
{
    int32_t left_milli_mps;
    int32_t right_milli_mps;

    memset(inputs, 0, sizeof(*inputs));
    inputs->uptime_ms = now_ms;

    /* No hardware E-stop input is mapped yet, so telemetry remains inhibited. */
    inputs->status_flags = HBALL_MSP_STATUS_ESTOP_ACTIVE;
    hball_can_update_wit_freshness(now_ms);
    if (g_hball_wit_accel_seen
        && g_hball_wit_gyro_seen
        && g_hball_wit_attitude_seen
        && ((uint32_t)(now_ms - g_hball_last_wit_accel_ms)
            <= HBALL_CAN_IMU_FRESH_MS)
        && ((uint32_t)(now_ms - g_hball_last_wit_gyro_ms)
            <= HBALL_CAN_IMU_FRESH_MS)
        && ((uint32_t)(now_ms - g_hball_last_wit_attitude_ms)
            <= HBALL_CAN_IMU_FRESH_MS))
    {
        inputs->status_flags |= HBALL_MSP_STATUS_IMU_VALID;
    }

    hball_can_snapshot_wit(inputs);

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
    g_hball_last_wit_accel_count = wit_accel_frame_count;
    g_hball_last_wit_gyro_count = wit_gyro_frame_count;
    g_hball_last_wit_attitude_count = wit_angle_frame_count;
    g_hball_last_wit_accel_ms = 0U;
    g_hball_last_wit_gyro_ms = 0U;
    g_hball_last_wit_attitude_ms = 0U;
    g_hball_wit_accel_seen = false;
    g_hball_wit_gyro_seen = false;
    g_hball_wit_attitude_seen = false;
    hball_can_recovery_init(&g_hball_can_recovery);

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
    uint16_t sequence;

    if (g_hball_can_stats.initialized == 0U)
    {
        return;
    }
    if (hball_can_recovery_should_attempt(
            &g_hball_can_recovery,
            now_ms,
            g_hball_can_stats.bus_off != 0U))
    {
        /*
         * TI DriverLib maps NORMAL to clearing MCAN_CCCR.INIT. This starts
         * the controller's ISO 11898-1 bus-off recovery sequence; it does
         * not bypass the protocol's required recessive-bit observation.
         */
        DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_NORMAL);
        g_hball_can_stats.bus_off_recovery_attempts++;
        return;
    }
    if ((g_hball_can_stats.bus_off != 0U)
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
    sequence = g_hball_sequences[stream];
    if (stream == HBALL_CAN_STREAM_ACCEL)
    {
        sequence = inputs.accel_source_sequence;
    }
    else if (stream == HBALL_CAN_STREAM_GYRO)
    {
        sequence = inputs.gyro_source_sequence;
    }
    else if (stream == HBALL_CAN_STREAM_ATTITUDE)
    {
        sequence = inputs.attitude_source_sequence;
    }
    if (!hball_can_encode_frame(
            stream, sequence, &inputs, &frame))
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
    if ((stream == HBALL_CAN_STREAM_WHEEL)
        || (stream == HBALL_CAN_STREAM_HEARTBEAT))
    {
        g_hball_sequences[stream]++;
    }
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
    uint8_t was_bus_off;

    if (DL_MCAN_getPendingInterrupt(MCAN0_INST) != DL_MCAN_IIDX_LINE1)
    {
        return;
    }
    interrupt_status = DL_MCAN_getIntrStatus(MCAN0_INST);
    was_bus_off = g_hball_can_stats.bus_off;
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
    if ((interrupt_status & (MCAN_IR_PEA_MASK | MCAN_IR_PED_MASK)) != 0U)
    {
        g_hball_can_stats.protocol_error_events++;
    }
    if ((interrupt_status & MCAN_IR_MRAF_MASK) != 0U)
    {
        g_hball_can_stats.message_ram_errors++;
    }
    hball_can_refresh_error_status();
    if ((interrupt_status & MCAN_IR_BO_MASK) != 0U)
    {
        if ((was_bus_off == 0U) && (g_hball_can_stats.bus_off != 0U))
        {
            g_hball_can_stats.bus_off_events++;
        }
        else if ((was_bus_off != 0U) && (g_hball_can_stats.bus_off == 0U))
        {
            g_hball_can_stats.bus_off_recoveries++;
        }
    }
}
