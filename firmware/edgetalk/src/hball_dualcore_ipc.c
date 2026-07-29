#include "hball_dualcore_ipc.h"

#include <math.h>
#include <stdatomic.h>
#include <string.h>

#define HBALL_IPC_CRC_SIZE 4U
#define HBALL_IPC_SENSOR_CRC_OFFSET \
    (HBALL_IPC_SENSOR_FRAME_SIZE - HBALL_IPC_CRC_SIZE)
#define HBALL_IPC_CONTROL_CRC_OFFSET \
    (HBALL_IPC_CONTROL_FRAME_SIZE - HBALL_IPC_CRC_SIZE)

_Static_assert(sizeof(float) == 4U, "dual-core IPC requires float32");
_Static_assert(
    _Alignof(hball_ipc_sensor_slot_t) == HBALL_IPC_CACHE_LINE_SIZE,
    "sensor slot must be one cache-line aligned"
);
_Static_assert(
    _Alignof(hball_ipc_control_slot_t) == HBALL_IPC_CACHE_LINE_SIZE,
    "control slot must be one cache-line aligned"
);
_Static_assert(
    sizeof(hball_ipc_shared_region_t) == HBALL_IPC_SHARED_REGION_SIZE,
    "shared region ABI changed"
);

static uint16_t hball_load_u16(const uint8_t *source)
{
    return (uint16_t)((uint16_t)source[0]
        | ((uint16_t)source[1] << 8U));
}

static uint32_t hball_load_u32(const uint8_t *source)
{
    return (uint32_t)source[0]
        | ((uint32_t)source[1] << 8U)
        | ((uint32_t)source[2] << 16U)
        | ((uint32_t)source[3] << 24U);
}

static uint64_t hball_load_u64(const uint8_t *source)
{
    return (uint64_t)hball_load_u32(source)
        | ((uint64_t)hball_load_u32(source + 4U) << 32U);
}

static float hball_load_float(const uint8_t *source)
{
    const uint32_t bits = hball_load_u32(source);
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void hball_store_u16(uint8_t *target, uint16_t value)
{
    target[0] = (uint8_t)value;
    target[1] = (uint8_t)(value >> 8U);
}

static void hball_store_u32(uint8_t *target, uint32_t value)
{
    target[0] = (uint8_t)value;
    target[1] = (uint8_t)(value >> 8U);
    target[2] = (uint8_t)(value >> 16U);
    target[3] = (uint8_t)(value >> 24U);
}

static void hball_store_u64(uint8_t *target, uint64_t value)
{
    hball_store_u32(target, (uint32_t)value);
    hball_store_u32(target + 4U, (uint32_t)(value >> 32U));
}

static void hball_store_float(uint8_t *target, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    hball_store_u32(target, bits);
}

static void hball_barrier(const hball_ipc_cache_ops_t *cache_ops)
{
    atomic_signal_fence(memory_order_seq_cst);
    if ((cache_ops != NULL) && (cache_ops->barrier != NULL))
    {
        cache_ops->barrier(cache_ops->context);
    }
    atomic_signal_fence(memory_order_seq_cst);
}

static void hball_clean(
    const hball_ipc_cache_ops_t *cache_ops, void *address, size_t length
)
{
    if ((cache_ops != NULL) && (cache_ops->clean != NULL))
    {
        cache_ops->clean(address, length, cache_ops->context);
    }
}

static void hball_invalidate(
    const hball_ipc_cache_ops_t *cache_ops, void *address, size_t length
)
{
    if ((cache_ops != NULL) && (cache_ops->invalidate != NULL))
    {
        cache_ops->invalidate(address, length, cache_ops->context);
    }
}

uint32_t hball_ipc_crc32c(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_C(0xffffffff);

    if ((data == NULL) && (length != 0U))
    {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (unsigned bit = 0U; bit < 8U; ++bit)
        {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);

            crc = (crc >> 1U) ^ (UINT32_C(0x82f63b78) & mask);
        }
    }
    return crc ^ UINT32_C(0xffffffff);
}

static bool hball_sensor_is_valid(const hball_sensor_snapshot_t *snapshot)
{
    return isfinite(snapshot->ball_position_m)
        && isfinite(snapshot->vision_confidence)
        && isfinite(snapshot->longitudinal_accel_mps2)
        && isfinite(snapshot->lateral_accel_mps2)
        && isfinite(snapshot->body_pitch_rad)
        && isfinite(snapshot->yaw_rate_rad_s)
        && isfinite(snapshot->body_speed_mps)
        && isfinite(snapshot->motor_angle_rad)
        && isfinite(snapshot->motor_velocity_rad_s)
        && isfinite(snapshot->motor_torque_nm)
        && (snapshot->vision_confidence >= 0.0F)
        && (snapshot->vision_confidence <= 1.0F);
}

static bool hball_control_is_valid(const hball_control_shadow_t *shadow)
{
    const uint16_t known_flags = HBALL_IPC_CONTROL_FLAG_SAFETY_ELIGIBLE
        | HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY;

    return isfinite(shadow->target_angle_rad)
        && isfinite(shadow->estimated_position_m)
        && isfinite(shadow->estimated_velocity_mps)
        && isfinite(shadow->estimated_disturbance_mps2)
        && (shadow->mode <= HBALL_IPC_CONTROL_MODE_MAX)
        && ((shadow->flags & (uint16_t)~known_flags) == 0U)
        && ((shadow->flags & HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY) != 0U);
}

static void hball_encode_header(
    uint8_t *frame,
    uint16_t message_type,
    uint16_t frame_size,
    uint16_t payload_size,
    uint32_t producer_sequence,
    uint32_t produced_time_ms,
    uint32_t valid_flags
)
{
    hball_store_u32(frame + 4U, HBALL_IPC_MAGIC);
    hball_store_u16(frame + 8U, HBALL_IPC_VERSION);
    hball_store_u16(frame + 10U, message_type);
    hball_store_u16(frame + 12U, frame_size);
    hball_store_u16(frame + 14U, payload_size);
    hball_store_u32(frame + 16U, producer_sequence);
    hball_store_u32(frame + 20U, produced_time_ms);
    hball_store_u32(frame + 24U, valid_flags);
    hball_store_u32(frame + 28U, 0U);
}

static bool hball_header_is_valid(
    const uint8_t *frame,
    uint16_t message_type,
    uint16_t frame_size,
    uint16_t payload_size
)
{
    return (hball_load_u32(frame + 4U) == HBALL_IPC_MAGIC)
        && (hball_load_u16(frame + 8U) == HBALL_IPC_VERSION)
        && (hball_load_u16(frame + 10U) == message_type)
        && (hball_load_u16(frame + 12U) == frame_size)
        && (hball_load_u16(frame + 14U) == payload_size)
        && (hball_load_u32(frame + 28U) == 0U);
}

static void hball_publish_frame(
    uint8_t *shared,
    const uint8_t *staging,
    size_t frame_size,
    const hball_ipc_cache_ops_t *cache_ops
)
{
    const uint32_t previous = hball_load_u32(shared);
    const uint32_t writing = (previous & UINT32_C(0xfffffffe)) + 1U;
    const uint32_t committed = writing + 1U;

    hball_store_u32(shared, writing);
    hball_barrier(cache_ops);
    hball_clean(cache_ops, shared, HBALL_IPC_CACHE_LINE_SIZE);

    memcpy(shared + 4U, staging + 4U, frame_size - 4U);
    hball_barrier(cache_ops);
    hball_clean(cache_ops, shared, frame_size);

    hball_store_u32(shared, committed);
    hball_barrier(cache_ops);
    hball_clean(cache_ops, shared, HBALL_IPC_CACHE_LINE_SIZE);
}

static hball_ipc_result_t hball_read_frame(
    const uint8_t *shared,
    uint8_t *staging,
    size_t frame_size,
    const hball_ipc_cache_ops_t *cache_ops
)
{
    uint32_t sequence_before;
    uint32_t sequence_after;

    hball_invalidate(cache_ops, (void *)shared, frame_size);
    hball_barrier(cache_ops);
    sequence_before = hball_load_u32(shared);
    if ((sequence_before & 1U) != 0U)
    {
        return HBALL_IPC_BUSY;
    }
    memcpy(staging, shared, frame_size);
    hball_barrier(cache_ops);
    hball_invalidate(
        cache_ops, (void *)shared, HBALL_IPC_CACHE_LINE_SIZE
    );
    hball_barrier(cache_ops);
    sequence_after = hball_load_u32(shared);
    if ((sequence_before != sequence_after) || ((sequence_after & 1U) != 0U))
    {
        return HBALL_IPC_BUSY;
    }
    return HBALL_IPC_OK;
}

void hball_ipc_region_reset(
    hball_ipc_shared_region_t *region, const hball_ipc_cache_ops_t *cache_ops
)
{
    if (region == NULL)
    {
        return;
    }
    memset(region, 0, sizeof(*region));
    hball_barrier(cache_ops);
    hball_clean(cache_ops, region, sizeof(*region));
}

hball_ipc_result_t hball_ipc_sensor_publish(
    hball_ipc_sensor_slot_t *slot,
    const hball_sensor_snapshot_t *snapshot,
    const hball_ipc_cache_ops_t *cache_ops
)
{
    uint8_t frame[HBALL_IPC_SENSOR_FRAME_SIZE] = {0};
    uint8_t *payload = frame + HBALL_IPC_HEADER_SIZE;

    if ((slot == NULL) || (snapshot == NULL))
    {
        return HBALL_IPC_ARGUMENT;
    }
    if (!hball_sensor_is_valid(snapshot))
    {
        return HBALL_IPC_RANGE;
    }
    hball_encode_header(
        frame,
        HBALL_IPC_SENSOR_MESSAGE_TYPE,
        HBALL_IPC_SENSOR_FRAME_SIZE,
        HBALL_IPC_SENSOR_PAYLOAD_SIZE,
        snapshot->sequence,
        snapshot->created_time_ms,
        snapshot->valid_flags
    );
    hball_store_u32(payload + 0U, snapshot->vision_sequence);
    hball_store_u16(payload + 4U, snapshot->accel_sequence);
    hball_store_u16(payload + 6U, snapshot->gyro_sequence);
    hball_store_u16(payload + 8U, snapshot->attitude_sequence);
    hball_store_u16(payload + 10U, snapshot->wheel_sequence);
    hball_store_u16(payload + 12U, snapshot->msp_status_flags);
    payload[14] = snapshot->motor_fault_summary;
    hball_store_u64(payload + 16U, snapshot->vision_capture_time_us);
    hball_store_u32(payload + 24U, snapshot->vision_receive_age_ms);
    hball_store_u32(payload + 28U, snapshot->imu_age_ms);
    hball_store_u32(payload + 32U, snapshot->wheel_age_ms);
    hball_store_u32(payload + 36U, snapshot->motor_age_ms);
    hball_store_u32(payload + 40U, snapshot->heartbeat_age_ms);
    hball_store_float(payload + 44U, snapshot->ball_position_m);
    hball_store_float(payload + 48U, snapshot->vision_confidence);
    hball_store_float(payload + 52U, snapshot->longitudinal_accel_mps2);
    hball_store_float(payload + 56U, snapshot->lateral_accel_mps2);
    hball_store_float(payload + 60U, snapshot->body_pitch_rad);
    hball_store_float(payload + 64U, snapshot->yaw_rate_rad_s);
    hball_store_float(payload + 68U, snapshot->body_speed_mps);
    hball_store_float(payload + 72U, snapshot->motor_angle_rad);
    hball_store_float(payload + 76U, snapshot->motor_velocity_rad_s);
    hball_store_float(payload + 80U, snapshot->motor_torque_nm);
    hball_store_u32(
        frame + HBALL_IPC_SENSOR_CRC_OFFSET,
        hball_ipc_crc32c(
            frame + 4U, HBALL_IPC_SENSOR_FRAME_SIZE - 8U
        )
    );
    hball_publish_frame(
        slot->bytes, frame, HBALL_IPC_SENSOR_FRAME_SIZE, cache_ops
    );
    return HBALL_IPC_OK;
}

hball_ipc_result_t hball_ipc_sensor_read(
    const hball_ipc_sensor_slot_t *slot,
    hball_sensor_snapshot_t *snapshot,
    const hball_ipc_cache_ops_t *cache_ops
)
{
    uint8_t frame[HBALL_IPC_SENSOR_FRAME_SIZE];
    const uint8_t *payload = frame + HBALL_IPC_HEADER_SIZE;
    hball_ipc_result_t result;

    if ((slot == NULL) || (snapshot == NULL))
    {
        return HBALL_IPC_ARGUMENT;
    }
    result = hball_read_frame(
        slot->bytes, frame, HBALL_IPC_SENSOR_FRAME_SIZE, cache_ops
    );
    if (result != HBALL_IPC_OK)
    {
        return result;
    }
    if (!hball_header_is_valid(
            frame,
            HBALL_IPC_SENSOR_MESSAGE_TYPE,
            HBALL_IPC_SENSOR_FRAME_SIZE,
            HBALL_IPC_SENSOR_PAYLOAD_SIZE))
    {
        return HBALL_IPC_HEADER;
    }
    if (hball_ipc_crc32c(frame + 4U, HBALL_IPC_SENSOR_FRAME_SIZE - 8U)
        != hball_load_u32(frame + HBALL_IPC_SENSOR_CRC_OFFSET))
    {
        return HBALL_IPC_CRC;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->sequence = hball_load_u32(frame + 16U);
    snapshot->created_time_ms = hball_load_u32(frame + 20U);
    snapshot->valid_flags = hball_load_u32(frame + 24U);
    snapshot->vision_sequence = hball_load_u32(payload + 0U);
    snapshot->accel_sequence = hball_load_u16(payload + 4U);
    snapshot->gyro_sequence = hball_load_u16(payload + 6U);
    snapshot->attitude_sequence = hball_load_u16(payload + 8U);
    snapshot->wheel_sequence = hball_load_u16(payload + 10U);
    snapshot->msp_status_flags = hball_load_u16(payload + 12U);
    snapshot->motor_fault_summary = payload[14];
    snapshot->vision_capture_time_us = hball_load_u64(payload + 16U);
    snapshot->vision_receive_age_ms = hball_load_u32(payload + 24U);
    snapshot->imu_age_ms = hball_load_u32(payload + 28U);
    snapshot->wheel_age_ms = hball_load_u32(payload + 32U);
    snapshot->motor_age_ms = hball_load_u32(payload + 36U);
    snapshot->heartbeat_age_ms = hball_load_u32(payload + 40U);
    snapshot->ball_position_m = hball_load_float(payload + 44U);
    snapshot->vision_confidence = hball_load_float(payload + 48U);
    snapshot->longitudinal_accel_mps2 = hball_load_float(payload + 52U);
    snapshot->lateral_accel_mps2 = hball_load_float(payload + 56U);
    snapshot->body_pitch_rad = hball_load_float(payload + 60U);
    snapshot->yaw_rate_rad_s = hball_load_float(payload + 64U);
    snapshot->body_speed_mps = hball_load_float(payload + 68U);
    snapshot->motor_angle_rad = hball_load_float(payload + 72U);
    snapshot->motor_velocity_rad_s = hball_load_float(payload + 76U);
    snapshot->motor_torque_nm = hball_load_float(payload + 80U);
    return hball_sensor_is_valid(snapshot) ? HBALL_IPC_OK : HBALL_IPC_RANGE;
}

hball_ipc_result_t hball_ipc_control_publish(
    hball_ipc_control_slot_t *slot,
    const hball_control_shadow_t *shadow,
    const hball_ipc_cache_ops_t *cache_ops
)
{
    uint8_t frame[HBALL_IPC_CONTROL_FRAME_SIZE] = {0};
    uint8_t *payload = frame + HBALL_IPC_HEADER_SIZE;

    if ((slot == NULL) || (shadow == NULL))
    {
        return HBALL_IPC_ARGUMENT;
    }
    if (!hball_control_is_valid(shadow))
    {
        return HBALL_IPC_RANGE;
    }
    hball_encode_header(
        frame,
        HBALL_IPC_CONTROL_MESSAGE_TYPE,
        HBALL_IPC_CONTROL_FRAME_SIZE,
        HBALL_IPC_CONTROL_PAYLOAD_SIZE,
        shadow->controller_steps,
        shadow->produced_time_ms,
        shadow->flags
    );
    hball_store_u32(payload + 0U, shadow->source_sensor_sequence);
    hball_store_u32(payload + 4U, shadow->controller_steps);
    hball_store_u32(payload + 8U, shadow->deadline_misses);
    hball_store_u16(payload + 12U, shadow->mode);
    hball_store_u16(payload + 14U, shadow->flags);
    hball_store_float(payload + 16U, shadow->target_angle_rad);
    hball_store_float(payload + 20U, shadow->estimated_position_m);
    hball_store_float(payload + 24U, shadow->estimated_velocity_mps);
    hball_store_float(payload + 28U, shadow->estimated_disturbance_mps2);
    hball_store_u32(
        frame + HBALL_IPC_CONTROL_CRC_OFFSET,
        hball_ipc_crc32c(
            frame + 4U, HBALL_IPC_CONTROL_FRAME_SIZE - 8U
        )
    );
    hball_publish_frame(
        slot->bytes, frame, HBALL_IPC_CONTROL_FRAME_SIZE, cache_ops
    );
    return HBALL_IPC_OK;
}

hball_ipc_result_t hball_ipc_control_read(
    const hball_ipc_control_slot_t *slot,
    hball_control_shadow_t *shadow,
    const hball_ipc_cache_ops_t *cache_ops
)
{
    uint8_t frame[HBALL_IPC_CONTROL_FRAME_SIZE];
    const uint8_t *payload = frame + HBALL_IPC_HEADER_SIZE;
    hball_ipc_result_t result;

    if ((slot == NULL) || (shadow == NULL))
    {
        return HBALL_IPC_ARGUMENT;
    }
    result = hball_read_frame(
        slot->bytes, frame, HBALL_IPC_CONTROL_FRAME_SIZE, cache_ops
    );
    if (result != HBALL_IPC_OK)
    {
        return result;
    }
    if (!hball_header_is_valid(
            frame,
            HBALL_IPC_CONTROL_MESSAGE_TYPE,
            HBALL_IPC_CONTROL_FRAME_SIZE,
            HBALL_IPC_CONTROL_PAYLOAD_SIZE))
    {
        return HBALL_IPC_HEADER;
    }
    if (hball_ipc_crc32c(frame + 4U, HBALL_IPC_CONTROL_FRAME_SIZE - 8U)
        != hball_load_u32(frame + HBALL_IPC_CONTROL_CRC_OFFSET))
    {
        return HBALL_IPC_CRC;
    }

    memset(shadow, 0, sizeof(*shadow));
    shadow->source_sensor_sequence = hball_load_u32(payload + 0U);
    shadow->controller_steps = hball_load_u32(payload + 4U);
    shadow->deadline_misses = hball_load_u32(payload + 8U);
    shadow->produced_time_ms = hball_load_u32(frame + 20U);
    shadow->mode = hball_load_u16(payload + 12U);
    shadow->flags = hball_load_u16(payload + 14U);
    shadow->target_angle_rad = hball_load_float(payload + 16U);
    shadow->estimated_position_m = hball_load_float(payload + 20U);
    shadow->estimated_velocity_mps = hball_load_float(payload + 24U);
    shadow->estimated_disturbance_mps2 = hball_load_float(payload + 28U);
    return hball_control_is_valid(shadow) ? HBALL_IPC_OK : HBALL_IPC_RANGE;
}
