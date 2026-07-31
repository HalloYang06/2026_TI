#include "hball_can.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define HBALL_MSP_REBOOT_BACKSTEP_MIN_MS 250U

static uint16_t hball_u16_from_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static uint16_t hball_u16_from_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static uint32_t hball_u32_from_le(const uint8_t *data)
{
    return (uint32_t)data[0]
        | ((uint32_t)data[1] << 8U)
        | ((uint32_t)data[2] << 16U)
        | ((uint32_t)data[3] << 24U);
}

static float hball_float_from_le(const uint8_t *data)
{
    const uint32_t bits = hball_u32_from_le(data);
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

static float hball_i16_milli_to_float(const uint8_t *data)
{
    const int16_t value = (int16_t)hball_u16_from_le(data);

    return (float)value * 0.001F;
}

static float hball_uint16_to_float(uint16_t value, float minimum, float maximum)
{
    return (float)value * (maximum - minimum) / 65535.0F + minimum;
}

uint32_t hball_rs00_ext_id(uint8_t comm_type, uint16_t data2, uint8_t data1)
{
    return (((uint32_t)comm_type & 0x1FUL) << 24)
        | (((uint32_t)data2 & 0xFFFFUL) << 8)
        | ((uint32_t)data1 & 0xFFUL);
}

void hball_motor_monitor_init(hball_motor_monitor_t *monitor, uint8_t motor_id)
{
    if (monitor == NULL)
    {
        return;
    }
    memset(monitor, 0, sizeof(*monitor));
    monitor->motor_id = motor_id;
}

bool hball_motor_monitor_make_probe(
    hball_motor_monitor_t *monitor, hball_can_frame_t *frame
)
{
    if ((monitor == NULL) || (frame == NULL) || (monitor->motor_id == 0U))
    {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->id = hball_rs00_ext_id(
        HBALL_RS00_TYPE_GET_ID, HBALL_RS00_MASTER_ID, monitor->motor_id
    );
    frame->is_extended = 1U;
    frame->dlc = 8U;
    monitor->probe_pending = true;
    monitor->probe_valid = false;
    return true;
}

static int hball_rs00_parameter_slot(uint16_t index)
{
    switch (index)
    {
    case HBALL_RS00_INDEX_RUN_MODE:
        return 0;
    case HBALL_RS00_INDEX_MECH_POSITION:
        return 1;
    case HBALL_RS00_INDEX_FILTERED_IQ:
        return 2;
    case HBALL_RS00_INDEX_MECH_VELOCITY:
        return 3;
    case HBALL_RS00_INDEX_VBUS:
        return 4;
    case HBALL_RS00_INDEX_ROTATION:
        return 5;
    default:
        return -1;
    }
}

bool hball_rs00_parameter_is_read_only(uint16_t index)
{
    return hball_rs00_parameter_slot(index) >= 0;
}

bool hball_motor_monitor_make_parameter_read(
    hball_motor_monitor_t *monitor,
    uint16_t index,
    uint32_t now_ms,
    hball_can_frame_t *frame
)
{
    if ((monitor == NULL) || (frame == NULL) || (monitor->motor_id == 0U)
        || monitor->parameter_pending
        || !hball_rs00_parameter_is_read_only(index))
    {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->id = hball_rs00_ext_id(
        HBALL_RS00_TYPE_GET_SINGLE_PARAMETER,
        HBALL_RS00_MASTER_ID,
        monitor->motor_id
    );
    frame->is_extended = 1U;
    frame->dlc = 8U;
    frame->data[0] = (uint8_t)index;
    frame->data[1] = (uint8_t)(index >> 8U);
    monitor->parameter_pending = true;
    monitor->pending_parameter_index = index;
    monitor->last_parameter_request_ms = now_ms;
    return true;
}

bool hball_motor_monitor_expire_parameter_request(
    hball_motor_monitor_t *monitor,
    uint32_t now_ms,
    uint32_t timeout_ms
)
{
    if ((monitor == NULL) || !monitor->parameter_pending
        || ((uint32_t)(now_ms - monitor->last_parameter_request_ms)
            <= timeout_ms))
    {
        return false;
    }
    monitor->parameter_pending = false;
    monitor->pending_parameter_index = 0U;
    monitor->parameter_timeout_total++;
    return true;
}

static hball_can_event_t hball_accept_probe_reply(
    hball_motor_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint16_t data2,
    uint32_t now_ms
)
{
    uint64_t unique_id = 0U;
    uint8_t index;
    const uint8_t reply_id = (uint8_t)(frame->id & 0xFFU);
    const uint8_t motor_id = (uint8_t)(data2 & 0xFFU);

    if (frame->dlc != 8U)
    {
        return HBALL_CAN_EVENT_INVALID;
    }
    if ((reply_id != HBALL_RS00_GET_ID_REPLY)
        || !monitor->probe_pending
        || (motor_id != monitor->motor_id))
    {
        return HBALL_CAN_EVENT_IGNORED;
    }

    for (index = 0U; index < 8U; ++index)
    {
        unique_id |= (uint64_t)frame->data[index] << (8U * index);
    }
    monitor->unique_id = unique_id;
    monitor->probe_valid = true;
    monitor->probe_pending = false;
    monitor->last_probe_ms = now_ms;
    return HBALL_CAN_EVENT_PROBE_REPLY;
}

static hball_can_event_t hball_accept_feedback(
    hball_motor_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint16_t data2,
    uint32_t now_ms
)
{
    const uint8_t host_id = (uint8_t)(frame->id & 0xFFU);
    const uint8_t motor_id = (uint8_t)(data2 & 0xFFU);

    if ((host_id != HBALL_RS00_MASTER_ID) || (motor_id != monitor->motor_id))
    {
        return HBALL_CAN_EVENT_IGNORED;
    }
    if (frame->dlc != 8U)
    {
        return HBALL_CAN_EVENT_INVALID;
    }

    monitor->feedback.motor_id = motor_id;
    monitor->feedback.fault_summary = (uint8_t)((data2 >> 8) & 0x3FU);
    monitor->feedback.mode_state = (uint8_t)((data2 >> 14) & 0x03U);
    monitor->feedback.position_rad = hball_uint16_to_float(
        hball_u16_from_be(&frame->data[0]),
        HBALL_RS00_POSITION_MIN_RAD,
        HBALL_RS00_POSITION_MAX_RAD
    );
    monitor->feedback.velocity_rad_s = hball_uint16_to_float(
        hball_u16_from_be(&frame->data[2]),
        HBALL_RS00_VELOCITY_MIN_RAD_S,
        HBALL_RS00_VELOCITY_MAX_RAD_S
    );
    monitor->feedback.torque_nm = hball_uint16_to_float(
        hball_u16_from_be(&frame->data[4]),
        HBALL_RS00_TORQUE_MIN_NM,
        HBALL_RS00_TORQUE_MAX_NM
    );
    monitor->feedback.temperature_c = (float)hball_u16_from_be(&frame->data[6]) / 10.0F;
    monitor->feedback_valid = true;
    monitor->last_feedback_ms = now_ms;
    return HBALL_CAN_EVENT_FEEDBACK;
}

static hball_can_event_t hball_accept_parameter_reply(
    hball_motor_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint16_t data2,
    uint32_t now_ms
)
{
    const uint8_t host_id = (uint8_t)(frame->id & 0xFFU);
    const uint8_t motor_id = (uint8_t)(data2 & 0xFFU);
    uint16_t index;
    int slot;
    float float_value = 0.0F;

    if ((host_id != HBALL_RS00_MASTER_ID) || (motor_id != monitor->motor_id))
    {
        return HBALL_CAN_EVENT_IGNORED;
    }
    if (frame->dlc != 8U)
    {
        return HBALL_CAN_EVENT_INVALID;
    }
    index = hball_u16_from_le(frame->data);
    slot = hball_rs00_parameter_slot(index);
    if (slot < 0)
    {
        return HBALL_CAN_EVENT_INVALID;
    }
    if (!monitor->parameter_pending
        || (index != monitor->pending_parameter_index))
    {
        return HBALL_CAN_EVENT_IGNORED;
    }
    if ((index != HBALL_RS00_INDEX_RUN_MODE)
        && (index != HBALL_RS00_INDEX_ROTATION))
    {
        float_value = hball_float_from_le(frame->data + 4U);
        if (!isfinite(float_value))
        {
            return HBALL_CAN_EVENT_INVALID;
        }
    }

    switch (index)
    {
    case HBALL_RS00_INDEX_RUN_MODE:
        if (frame->data[4] > 5U)
        {
            return HBALL_CAN_EVENT_INVALID;
        }
        monitor->parameters.run_mode = frame->data[4];
        break;
    case HBALL_RS00_INDEX_MECH_POSITION:
        monitor->parameters.mech_position_rad = float_value;
        break;
    case HBALL_RS00_INDEX_FILTERED_IQ:
        monitor->parameters.filtered_iq_a = float_value;
        break;
    case HBALL_RS00_INDEX_MECH_VELOCITY:
        monitor->parameters.mech_velocity_rad_s = float_value;
        break;
    case HBALL_RS00_INDEX_VBUS:
        monitor->parameters.vbus_v = float_value;
        break;
    default:
        monitor->parameters.rotation = (int16_t)hball_u16_from_le(
            frame->data + 4U
        );
        break;
    }
    monitor->parameters.valid_flags |= (uint8_t)(UINT8_C(1) << slot);
    monitor->parameters.last_update_ms[slot] = now_ms;
    monitor->parameter_pending = false;
    monitor->pending_parameter_index = 0U;
    monitor->parameter_rx_total++;
    return HBALL_CAN_EVENT_PARAMETER;
}

static hball_can_event_t hball_accept_error_raw(
    hball_motor_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint16_t data2
)
{
    const uint8_t host_id = (uint8_t)(frame->id & 0xFFU);
    const uint8_t motor_id = (uint8_t)(data2 & 0xFFU);

    if ((host_id != HBALL_RS00_MASTER_ID) || (motor_id != monitor->motor_id))
    {
        return HBALL_CAN_EVENT_IGNORED;
    }
    if (frame->dlc > 8U)
    {
        return HBALL_CAN_EVENT_INVALID;
    }
    monitor->last_error_raw_id = frame->id;
    monitor->last_error_raw_dlc = frame->dlc;
    memset(monitor->last_error_raw_data, 0, sizeof(monitor->last_error_raw_data));
    memcpy(monitor->last_error_raw_data, frame->data, frame->dlc);
    monitor->error_raw_total++;
    return HBALL_CAN_EVENT_ERROR_RAW;
}

hball_can_event_t hball_motor_monitor_accept(
    hball_motor_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint32_t now_ms
)
{
    hball_can_event_t event;
    uint8_t comm_type;
    uint16_t data2;

    if ((monitor == NULL) || (frame == NULL))
    {
        return HBALL_CAN_EVENT_INVALID;
    }

    monitor->rx_total++;
    if ((frame->is_extended == 0U) || (frame->is_remote != 0U))
    {
        monitor->rx_ignored++;
        return HBALL_CAN_EVENT_IGNORED;
    }

    comm_type = (uint8_t)((frame->id >> 24) & 0x1FU);
    data2 = (uint16_t)((frame->id >> 8) & 0xFFFFU);
    if (comm_type == HBALL_RS00_TYPE_GET_ID)
    {
        event = hball_accept_probe_reply(monitor, frame, data2, now_ms);
    }
    else if ((comm_type == HBALL_RS00_TYPE_FEEDBACK)
        || (comm_type == HBALL_RS00_TYPE_ACTIVE_REPORT))
    {
        event = hball_accept_feedback(monitor, frame, data2, now_ms);
    }
    else if (comm_type == HBALL_RS00_TYPE_GET_SINGLE_PARAMETER)
    {
        event = hball_accept_parameter_reply(monitor, frame, data2, now_ms);
    }
    else if (comm_type == HBALL_RS00_TYPE_ERROR_FEEDBACK)
    {
        event = hball_accept_error_raw(monitor, frame, data2);
    }
    else
    {
        event = HBALL_CAN_EVENT_IGNORED;
    }

    if (event == HBALL_CAN_EVENT_INVALID)
    {
        monitor->rx_invalid++;
    }
    else if (event == HBALL_CAN_EVENT_IGNORED)
    {
        monitor->rx_ignored++;
    }
    return event;
}

bool hball_motor_monitor_feedback_fresh(
    const hball_motor_monitor_t *monitor,
    uint32_t now_ms,
    uint32_t timeout_ms
)
{
    return (monitor != NULL)
        && monitor->feedback_valid
        && ((uint32_t)(now_ms - monitor->last_feedback_ms) <= timeout_ms);
}

bool hball_motor_monitor_parameters_fresh(
    const hball_motor_monitor_t *monitor,
    uint32_t now_ms,
    uint32_t timeout_ms
)
{
    return (monitor != NULL)
        && hball_motor_parameters_fresh(
            &monitor->parameters, now_ms, timeout_ms
        );
}

bool hball_motor_parameters_fresh(
    const hball_motor_parameters_t *parameters,
    uint32_t now_ms,
    uint32_t timeout_ms
)
{
    uint8_t slot;

    if ((parameters == NULL)
        || (parameters->valid_flags != HBALL_RS00_PARAMETER_VALID_ALL))
    {
        return false;
    }
    for (slot = 0U; slot < HBALL_RS00_PARAMETER_COUNT; ++slot)
    {
        if ((uint32_t)(now_ms - parameters->last_update_ms[slot])
            > timeout_ms)
        {
            return false;
        }
    }
    return true;
}

void hball_msp_monitor_init(hball_msp_monitor_t *monitor)
{
    if (monitor != NULL)
    {
        memset(monitor, 0, sizeof(*monitor));
    }
}

static hball_msp_event_t hball_msp_decode_heartbeat(
    hball_msp_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint32_t now_ms
)
{
    monitor->heartbeat_sequence = hball_u16_from_le(frame->data);
    monitor->status_flags = hball_u16_from_le(frame->data + 2U);
    monitor->uptime_ms = hball_u32_from_le(frame->data + 4U);
    monitor->last_heartbeat_ms = now_ms;
    monitor->heartbeat_valid = true;
    return HBALL_MSP_EVENT_HEARTBEAT;
}

static hball_msp_event_t hball_msp_decode_vector(
    hball_msp_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint32_t now_ms,
    bool is_accel
)
{
    float *values = is_accel ? monitor->accel_mps2 : monitor->gyro_rad_s;

    values[0] = hball_i16_milli_to_float(frame->data + 2U);
    values[1] = hball_i16_milli_to_float(frame->data + 4U);
    values[2] = hball_i16_milli_to_float(frame->data + 6U);
    if (is_accel)
    {
        monitor->accel_sequence = hball_u16_from_le(frame->data);
        monitor->last_accel_ms = now_ms;
        monitor->accel_valid = true;
        return HBALL_MSP_EVENT_ACCEL;
    }
    monitor->gyro_sequence = hball_u16_from_le(frame->data);
    monitor->last_gyro_ms = now_ms;
    monitor->gyro_valid = true;
    return HBALL_MSP_EVENT_GYRO;
}

static hball_msp_event_t hball_msp_decode_wheel(
    hball_msp_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint32_t now_ms
)
{
    monitor->wheel_sequence = hball_u16_from_le(frame->data);
    monitor->wheel_left_mps = hball_i16_milli_to_float(frame->data + 2U);
    monitor->wheel_right_mps = hball_i16_milli_to_float(frame->data + 4U);
    monitor->body_speed_mps = hball_i16_milli_to_float(frame->data + 6U);
    monitor->last_wheel_ms = now_ms;
    monitor->wheel_valid = true;
    return HBALL_MSP_EVENT_WHEEL;
}

static hball_msp_event_t hball_msp_decode_attitude(
    hball_msp_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint32_t now_ms
)
{
    monitor->attitude_sequence = hball_u16_from_le(frame->data);
    monitor->attitude_rad[0] = hball_i16_milli_to_float(frame->data + 2U);
    monitor->attitude_rad[1] = hball_i16_milli_to_float(frame->data + 4U);
    monitor->attitude_rad[2] = hball_i16_milli_to_float(frame->data + 6U);
    monitor->last_attitude_ms = now_ms;
    monitor->attitude_valid = true;
    return HBALL_MSP_EVENT_ATTITUDE;
}

static hball_msp_event_t hball_msp_decode_imu_time(
    hball_msp_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint32_t now_ms
)
{
    monitor->imu_epoch = hball_u16_from_le(frame->data);
    monitor->imu_sample_mask = hball_u16_from_le(frame->data + 2U);
    monitor->imu_source_time_ms = hball_u32_from_le(frame->data + 4U);
    monitor->last_imu_time_ms = now_ms;
    monitor->imu_time_valid = true;
    return HBALL_MSP_EVENT_IMU_TIME;
}

typedef enum
{
    HBALL_MSP_SEQUENCE_ACCEPT = 0,
    HBALL_MSP_SEQUENCE_DUPLICATE,
    HBALL_MSP_SEQUENCE_OUT_OF_ORDER,
} hball_msp_sequence_result_t;

static hball_msp_sequence_result_t hball_msp_check_sequence(
    hball_msp_monitor_t *monitor,
    uint16_t previous,
    bool initialized,
    uint16_t sequence
)
{
    uint16_t delta;

    if (!initialized)
    {
        return HBALL_MSP_SEQUENCE_ACCEPT;
    }
    delta = (uint16_t)(sequence - previous);
    if (delta == 0U)
    {
        monitor->rx_duplicate++;
        return HBALL_MSP_SEQUENCE_DUPLICATE;
    }
    if (delta >= UINT16_C(0x8000))
    {
        monitor->rx_out_of_order++;
        return HBALL_MSP_SEQUENCE_OUT_OF_ORDER;
    }
    monitor->rx_gap += (uint32_t)delta - 1U;
    return HBALL_MSP_SEQUENCE_ACCEPT;
}

static hball_msp_sequence_result_t hball_msp_check_frame_sequence(
    hball_msp_monitor_t *monitor, const hball_can_frame_t *frame
)
{
    const uint16_t sequence = hball_u16_from_le(frame->data);

    switch (frame->id)
    {
    case HBALL_MSP_CAN_ID_HEARTBEAT:
        return hball_msp_check_sequence(
            monitor,
            monitor->heartbeat_sequence,
            monitor->heartbeat_valid,
            sequence
        );
    case HBALL_MSP_CAN_ID_ACCEL:
        return hball_msp_check_sequence(
            monitor,
            monitor->accel_sequence,
            monitor->accel_valid,
            sequence
        );
    case HBALL_MSP_CAN_ID_GYRO:
        return hball_msp_check_sequence(
            monitor,
            monitor->gyro_sequence,
            monitor->gyro_valid,
            sequence
        );
    case HBALL_MSP_CAN_ID_WHEEL:
        return hball_msp_check_sequence(
            monitor,
            monitor->wheel_sequence,
            monitor->wheel_valid,
            sequence
        );
    case HBALL_MSP_CAN_ID_IMU_TIME:
        return hball_msp_check_sequence(
            monitor,
            monitor->imu_epoch,
            monitor->imu_time_valid,
            sequence
        );
    default:
        return hball_msp_check_sequence(
            monitor,
            monitor->attitude_sequence,
            monitor->attitude_valid,
            sequence
        );
    }
}

static void hball_msp_accept_reboot_if_present(
    hball_msp_monitor_t *monitor, const hball_can_frame_t *frame
)
{
    uint32_t next_uptime_ms;
    uint32_t backward_ms;

    if ((frame->id != HBALL_MSP_CAN_ID_HEARTBEAT)
        || !monitor->heartbeat_valid)
    {
        return;
    }
    next_uptime_ms = hball_u32_from_le(frame->data + 4U);
    backward_ms = (uint32_t)(monitor->uptime_ms - next_uptime_ms);
    if (((uint32_t)(next_uptime_ms - monitor->uptime_ms)
            < UINT32_C(0x80000000))
        || (backward_ms < HBALL_MSP_REBOOT_BACKSTEP_MIN_MS))
    {
        return;
    }

    monitor->heartbeat_valid = false;
    monitor->accel_valid = false;
    monitor->gyro_valid = false;
    monitor->wheel_valid = false;
    monitor->attitude_valid = false;
    monitor->imu_time_valid = false;
    monitor->reboot_total++;
}

hball_msp_event_t hball_msp_monitor_accept(
    hball_msp_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint32_t now_ms
)
{
    hball_msp_event_t event;
    hball_msp_sequence_result_t sequence_result;
    const bool known_id = (frame != NULL)
        && ((frame->id == HBALL_MSP_CAN_ID_HEARTBEAT)
            || (frame->id == HBALL_MSP_CAN_ID_ACCEL)
            || (frame->id == HBALL_MSP_CAN_ID_GYRO)
            || (frame->id == HBALL_MSP_CAN_ID_WHEEL)
            || (frame->id == HBALL_MSP_CAN_ID_ATTITUDE)
            || (frame->id == HBALL_MSP_CAN_ID_IMU_TIME));

    if ((monitor == NULL) || (frame == NULL))
    {
        return HBALL_MSP_EVENT_INVALID;
    }
    monitor->rx_total++;
    if (frame->is_extended != 0U)
    {
        monitor->rx_ignored++;
        return HBALL_MSP_EVENT_IGNORED;
    }
    if (!known_id)
    {
        monitor->rx_ignored++;
        return HBALL_MSP_EVENT_IGNORED;
    }
    if ((frame->is_remote != 0U) || (frame->dlc != 8U))
    {
        monitor->rx_invalid++;
        return HBALL_MSP_EVENT_INVALID;
    }

    hball_msp_accept_reboot_if_present(monitor, frame);
    sequence_result = hball_msp_check_frame_sequence(monitor, frame);
    if (sequence_result == HBALL_MSP_SEQUENCE_DUPLICATE)
    {
        return HBALL_MSP_EVENT_DUPLICATE;
    }
    if (sequence_result == HBALL_MSP_SEQUENCE_OUT_OF_ORDER)
    {
        return HBALL_MSP_EVENT_OUT_OF_ORDER;
    }

    switch (frame->id)
    {
    case HBALL_MSP_CAN_ID_HEARTBEAT:
        event = hball_msp_decode_heartbeat(monitor, frame, now_ms);
        break;
    case HBALL_MSP_CAN_ID_ACCEL:
        event = hball_msp_decode_vector(monitor, frame, now_ms, true);
        break;
    case HBALL_MSP_CAN_ID_GYRO:
        event = hball_msp_decode_vector(monitor, frame, now_ms, false);
        break;
    case HBALL_MSP_CAN_ID_WHEEL:
        event = hball_msp_decode_wheel(monitor, frame, now_ms);
        break;
    case HBALL_MSP_CAN_ID_IMU_TIME:
        event = hball_msp_decode_imu_time(monitor, frame, now_ms);
        break;
    default:
        event = hball_msp_decode_attitude(monitor, frame, now_ms);
        break;
    }
    return event;
}

bool hball_msp_monitor_imu_fresh(
    const hball_msp_monitor_t *monitor,
    uint32_t now_ms,
    uint32_t timeout_ms
)
{
    return (monitor != NULL)
        && monitor->accel_valid
        && monitor->gyro_valid
        && monitor->attitude_valid
        && ((uint32_t)(now_ms - monitor->last_accel_ms) <= timeout_ms)
        && ((uint32_t)(now_ms - monitor->last_gyro_ms) <= timeout_ms)
        && ((uint32_t)(now_ms - monitor->last_attitude_ms) <= timeout_ms);
}

bool hball_msp_monitor_heartbeat_fresh(
    const hball_msp_monitor_t *monitor,
    uint32_t now_ms,
    uint32_t timeout_ms
)
{
    return (monitor != NULL)
        && monitor->heartbeat_valid
        && ((uint32_t)(now_ms - monitor->last_heartbeat_ms) <= timeout_ms);
}
