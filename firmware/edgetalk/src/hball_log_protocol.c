#include "hball_log_protocol.h"

#include <math.h>
#include <string.h>

static void put_u16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

static void put_u64(uint8_t *destination, uint64_t value)
{
    put_u32(destination, (uint32_t)value);
    put_u32(destination + 4U, (uint32_t)(value >> 32U));
}

static void put_float(uint8_t *destination, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    put_u32(destination, bits);
}

uint32_t hball_log_crc32c(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_MAX;

    if (data == NULL)
    {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (unsigned bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc >> 1) ^ ((crc & 1U) != 0U
                ? UINT32_C(0x82F63B78) : 0U);
        }
    }
    return ~crc;
}

bool hball_log_encode(
    const hball_log_record_t *record,
    uint8_t frame[HBALL_LOG_FRAME_SIZE]
)
{
    const float values[HBALL_LOG_FLOAT_COUNT] = {
        record != NULL ? record->ball_position_m : 0.0F,
        record != NULL ? record->vision_confidence : 0.0F,
        record != NULL ? record->estimated_position_m : 0.0F,
        record != NULL ? record->estimated_velocity_mps : 0.0F,
        record != NULL ? record->target_position_m : 0.0F,
        record != NULL ? record->estimated_disturbance_mps2 : 0.0F,
        record != NULL ? record->pipe_target_rad : 0.0F,
        record != NULL ? record->actual_pipe_angle_rad : 0.0F,
        record != NULL ? record->motor_target_rad : 0.0F,
        record != NULL ? record->motor_angle_rad : 0.0F,
        record != NULL ? record->motor_velocity_rad_s : 0.0F,
        record != NULL ? record->motor_torque_nm : 0.0F,
        record != NULL ? record->motor_temperature_c : 0.0F,
        record != NULL ? record->motor_filtered_iq_a : 0.0F,
        record != NULL ? record->motor_vbus_v : 0.0F,
        record != NULL ? record->accel_mps2[0] : 0.0F,
        record != NULL ? record->accel_mps2[1] : 0.0F,
        record != NULL ? record->accel_mps2[2] : 0.0F,
        record != NULL ? record->gyro_rad_s[0] : 0.0F,
        record != NULL ? record->gyro_rad_s[1] : 0.0F,
        record != NULL ? record->gyro_rad_s[2] : 0.0F,
        record != NULL ? record->attitude_rad[0] : 0.0F,
        record != NULL ? record->attitude_rad[1] : 0.0F,
        record != NULL ? record->attitude_rad[2] : 0.0F,
        record != NULL ? record->wheel_left_mps : 0.0F,
        record != NULL ? record->wheel_right_mps : 0.0F,
        record != NULL ? record->body_speed_mps : 0.0F,
        record != NULL ? record->controller_position_error_m : 0.0F,
        record != NULL ? record->controller_integral_error_m_s : 0.0F,
        record != NULL ? record->controller_filtered_accel_mps2 : 0.0F,
        record != NULL ? record->controller_feedback_rad : 0.0F,
        record != NULL ? record->controller_feedforward_rad : 0.0F,
        record != NULL ? record->controller_requested_rad : 0.0F,
        record != NULL ? record->controller_command_rad : 0.0F,
        record != NULL ? record->controller_p_rad : 0.0F,
        record != NULL ? record->controller_i_rad : 0.0F,
        record != NULL ? record->controller_d_rad : 0.0F,
        record != NULL ? record->controller_command_limit_rad : 0.0F,
        record != NULL ? record->controller_rate_limit_rad_s : 0.0F,
    };

    if ((record == NULL) || (frame == NULL))
    {
        return false;
    }
    for (size_t index = 0U; index < sizeof(values) / sizeof(values[0]); ++index)
    {
        if (!isfinite(values[index]))
        {
            return false;
        }
    }
    memset(frame, 0, HBALL_LOG_FRAME_SIZE);
    put_u32(frame + 0U, HBALL_LOG_MAGIC);
    put_u16(frame + 4U, HBALL_LOG_VERSION);
    put_u16(frame + 6U, HBALL_LOG_FRAME_SIZE);
    put_u32(frame + 8U, record->sequence);
    put_u32(frame + 12U, record->produced_time_ms);
    put_u32(frame + 16U, record->sensor_sequence);
    put_u32(frame + 20U, record->controller_steps);
    put_u32(frame + 24U, record->status_flags);
    put_u32(frame + 28U, record->sensor_valid_flags);
    put_u16(frame + 32U, record->control_mode);
    put_u16(frame + 34U, record->guard_reason);
    put_u32(frame + 36U, record->vision_sequence);
    put_u64(frame + 40U, record->vision_capture_time_us);
    put_u32(frame + 48U, record->vision_receive_time_ms);
    put_u32(frame + 52U, record->vision_processing_time_us);
    put_u16(frame + 56U, record->imu_epoch);
    put_u16(frame + 58U, record->imu_sample_mask);
    put_u32(frame + 60U, record->imu_source_time_ms);
    put_u32(frame + 64U, record->accel_receive_time_ms);
    put_u32(frame + 68U, record->gyro_receive_time_ms);
    put_u32(frame + 72U, record->attitude_receive_time_ms);
    put_u32(frame + 76U, record->imu_sync_receive_time_ms);
    put_u16(frame + 80U, record->wheel_sequence);
    put_u16(frame + 82U, record->accel_sequence);
    put_u16(frame + 84U, record->gyro_sequence);
    put_u16(frame + 86U, record->attitude_sequence);
    put_u32(frame + 88U, record->wheel_receive_time_ms);
    put_u32(frame + 92U, record->motor_receive_time_ms);
    put_u16(frame + 96U, record->msp_status_flags);
    put_u16(frame + 98U, record->vision_flags);
    frame[100U] = record->motor_fault_summary;
    frame[101U] = record->motor_mode_state;
    frame[102U] = record->motor_run_mode;
    frame[103U] = record->reserved;
    put_u32(frame + 104U, record->control_output_flags);
    put_u32(frame + 108U, record->vision_age_ms);
    put_u32(frame + 112U, record->imu_age_ms);
    put_u32(frame + 116U, record->wheel_age_ms);
    put_u32(frame + 120U, record->motor_age_ms);
    put_u32(frame + 124U, record->heartbeat_age_ms);
    for (size_t index = 0U; index < sizeof(values) / sizeof(values[0]); ++index)
    {
        put_float(frame + 128U + index * 4U, values[index]);
    }
    put_u32(
        frame + HBALL_LOG_FRAME_SIZE - 4U,
        hball_log_crc32c(frame, HBALL_LOG_FRAME_SIZE - 4U)
    );
    return true;
}
