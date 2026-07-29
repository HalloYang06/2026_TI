#include "hball_dualcore_ipc.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    unsigned clean_total;
    unsigned invalidate_total;
    unsigned barrier_total;
    bool tear_on_second_invalidate;
} cache_probe_t;

static void cache_clean(void *address, size_t length, void *context)
{
    cache_probe_t *probe = context;

    assert(address != NULL);
    assert((((uintptr_t)address) % HBALL_IPC_CACHE_LINE_SIZE) == 0U);
    assert((length % HBALL_IPC_CACHE_LINE_SIZE) == 0U);
    probe->clean_total++;
}

static void cache_invalidate(void *address, size_t length, void *context)
{
    cache_probe_t *probe = context;

    assert(address != NULL);
    assert((((uintptr_t)address) % HBALL_IPC_CACHE_LINE_SIZE) == 0U);
    assert((length % HBALL_IPC_CACHE_LINE_SIZE) == 0U);
    probe->invalidate_total++;
    if (probe->tear_on_second_invalidate
        && (probe->invalidate_total == 2U))
    {
        uint8_t *bytes = address;

        bytes[0] ^= 2U;
    }
}

static void memory_barrier(void *context)
{
    cache_probe_t *probe = context;

    probe->barrier_total++;
}

static hball_ipc_cache_ops_t make_cache_ops(cache_probe_t *probe)
{
    hball_ipc_cache_ops_t ops = {
        .clean = cache_clean,
        .invalidate = cache_invalidate,
        .barrier = memory_barrier,
        .context = probe,
    };

    return ops;
}

static hball_sensor_snapshot_t make_sensor_snapshot(void)
{
    hball_sensor_snapshot_t snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.sequence = 0x10203040U;
    snapshot.created_time_ms = 123456U;
    snapshot.valid_flags = HBALL_SENSOR_VALID_VISION
        | HBALL_SENSOR_VALID_IMU
        | HBALL_SENSOR_VALID_WHEEL
        | HBALL_SENSOR_VALID_MOTOR
        | HBALL_SENSOR_VALID_HEARTBEAT;
    snapshot.vision_sequence = 0xaabbccddU;
    snapshot.accel_sequence = 101U;
    snapshot.gyro_sequence = 102U;
    snapshot.attitude_sequence = 103U;
    snapshot.wheel_sequence = 104U;
    snapshot.msp_status_flags = 0x1234U;
    snapshot.motor_fault_summary = 0U;
    snapshot.vision_capture_time_us = UINT64_C(0x0102030405060708);
    snapshot.vision_receive_age_ms = 4U;
    snapshot.imu_age_ms = 3U;
    snapshot.wheel_age_ms = 7U;
    snapshot.motor_age_ms = 2U;
    snapshot.heartbeat_age_ms = 9U;
    snapshot.ball_position_m = 0.0125F;
    snapshot.vision_confidence = 0.93F;
    snapshot.longitudinal_accel_mps2 = -1.25F;
    snapshot.lateral_accel_mps2 = 0.75F;
    snapshot.body_pitch_rad = 0.021F;
    snapshot.yaw_rate_rad_s = -0.42F;
    snapshot.body_speed_mps = 0.8F;
    snapshot.motor_angle_rad = -0.031F;
    snapshot.motor_velocity_rad_s = 1.6F;
    snapshot.motor_torque_nm = 0.12F;
    return snapshot;
}

static void assert_float_bits_equal(float lhs, float rhs)
{
    uint32_t left_bits;
    uint32_t right_bits;

    memcpy(&left_bits, &lhs, sizeof(left_bits));
    memcpy(&right_bits, &rhs, sizeof(right_bits));
    assert(left_bits == right_bits);
}

static void assert_sensor_equal(
    const hball_sensor_snapshot_t *expected,
    const hball_sensor_snapshot_t *actual
)
{
    assert(actual->sequence == expected->sequence);
    assert(actual->created_time_ms == expected->created_time_ms);
    assert(actual->valid_flags == expected->valid_flags);
    assert(actual->vision_sequence == expected->vision_sequence);
    assert(actual->accel_sequence == expected->accel_sequence);
    assert(actual->gyro_sequence == expected->gyro_sequence);
    assert(actual->attitude_sequence == expected->attitude_sequence);
    assert(actual->wheel_sequence == expected->wheel_sequence);
    assert(actual->msp_status_flags == expected->msp_status_flags);
    assert(actual->motor_fault_summary == expected->motor_fault_summary);
    assert(actual->vision_capture_time_us == expected->vision_capture_time_us);
    assert(actual->vision_receive_age_ms == expected->vision_receive_age_ms);
    assert(actual->imu_age_ms == expected->imu_age_ms);
    assert(actual->wheel_age_ms == expected->wheel_age_ms);
    assert(actual->motor_age_ms == expected->motor_age_ms);
    assert(actual->heartbeat_age_ms == expected->heartbeat_age_ms);
    assert_float_bits_equal(actual->ball_position_m, expected->ball_position_m);
    assert_float_bits_equal(
        actual->vision_confidence, expected->vision_confidence
    );
    assert_float_bits_equal(
        actual->longitudinal_accel_mps2,
        expected->longitudinal_accel_mps2
    );
    assert_float_bits_equal(
        actual->lateral_accel_mps2, expected->lateral_accel_mps2
    );
    assert_float_bits_equal(actual->body_pitch_rad, expected->body_pitch_rad);
    assert_float_bits_equal(actual->yaw_rate_rad_s, expected->yaw_rate_rad_s);
    assert_float_bits_equal(actual->body_speed_mps, expected->body_speed_mps);
    assert_float_bits_equal(actual->motor_angle_rad, expected->motor_angle_rad);
    assert_float_bits_equal(
        actual->motor_velocity_rad_s, expected->motor_velocity_rad_s
    );
    assert_float_bits_equal(actual->motor_torque_nm, expected->motor_torque_nm);
}

static void test_layout_and_crc(void)
{
    static const uint8_t check[] = "123456789";

    assert(_Alignof(hball_ipc_sensor_slot_t) == HBALL_IPC_CACHE_LINE_SIZE);
    assert(_Alignof(hball_ipc_control_slot_t) == HBALL_IPC_CACHE_LINE_SIZE);
    assert(_Alignof(hball_ipc_shared_region_t) == HBALL_IPC_CACHE_LINE_SIZE);
    assert(sizeof(hball_ipc_sensor_slot_t) == HBALL_IPC_SENSOR_FRAME_SIZE);
    assert(sizeof(hball_ipc_control_slot_t) == HBALL_IPC_CONTROL_FRAME_SIZE);
    assert(sizeof(hball_ipc_shared_region_t) == HBALL_IPC_SHARED_REGION_SIZE);
    assert(hball_ipc_crc32c(check, sizeof(check) - 1U) == UINT32_C(0xe3069283));
}

static void test_sensor_round_trip_and_cache_contract(void)
{
    hball_ipc_shared_region_t region;
    hball_sensor_snapshot_t expected = make_sensor_snapshot();
    hball_sensor_snapshot_t actual;
    cache_probe_t probe = {0};
    hball_ipc_cache_ops_t ops = make_cache_ops(&probe);

    memset(&region, 0xa5, sizeof(region));
    hball_ipc_region_reset(&region, &ops);
    assert(probe.clean_total == 1U);
    assert(hball_ipc_sensor_publish(&region.sensor, &expected, &ops)
        == HBALL_IPC_OK);
    assert(probe.clean_total == 4U);
    assert(hball_ipc_sensor_read(&region.sensor, &actual, &ops)
        == HBALL_IPC_OK);
    assert(probe.invalidate_total == 2U);
    assert_sensor_equal(&expected, &actual);
}

static void test_control_round_trip(void)
{
    hball_ipc_control_slot_t slot = {0};
    hball_control_shadow_t expected = {
        .source_sensor_sequence = 41U,
        .controller_steps = 88U,
        .deadline_misses = 2U,
        .produced_time_ms = 98765U,
        .mode = 1U,
        .flags = HBALL_IPC_CONTROL_FLAG_SAFETY_ELIGIBLE
            | HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY,
        .target_angle_rad = 0.032F,
        .estimated_position_m = -0.014F,
        .estimated_velocity_mps = 0.23F,
        .estimated_disturbance_mps2 = -0.8F,
    };
    hball_control_shadow_t actual;

    assert(hball_ipc_control_publish(&slot, &expected, NULL) == HBALL_IPC_OK);
    assert(hball_ipc_control_read(&slot, &actual, NULL) == HBALL_IPC_OK);
    assert(actual.source_sensor_sequence == expected.source_sensor_sequence);
    assert(actual.controller_steps == expected.controller_steps);
    assert(actual.deadline_misses == expected.deadline_misses);
    assert(actual.produced_time_ms == expected.produced_time_ms);
    assert(actual.mode == expected.mode);
    assert(actual.flags == expected.flags);
    assert_float_bits_equal(actual.target_angle_rad, expected.target_angle_rad);
    assert_float_bits_equal(
        actual.estimated_position_m, expected.estimated_position_m
    );
    assert_float_bits_equal(
        actual.estimated_velocity_mps, expected.estimated_velocity_mps
    );
    assert_float_bits_equal(
        actual.estimated_disturbance_mps2,
        expected.estimated_disturbance_mps2
    );
}

static void test_corruption_busy_and_torn_read_fail_closed(void)
{
    hball_ipc_sensor_slot_t slot = {0};
    hball_sensor_snapshot_t expected = make_sensor_snapshot();
    hball_sensor_snapshot_t actual;
    cache_probe_t probe = {0};
    hball_ipc_cache_ops_t ops = make_cache_ops(&probe);

    assert(hball_ipc_sensor_publish(&slot, &expected, NULL) == HBALL_IPC_OK);
    slot.bytes[HBALL_IPC_HEADER_SIZE] ^= 0x80U;
    assert(hball_ipc_sensor_read(&slot, &actual, NULL) == HBALL_IPC_CRC);

    assert(hball_ipc_sensor_publish(&slot, &expected, NULL) == HBALL_IPC_OK);
    slot.bytes[4] ^= 0x01U;
    assert(hball_ipc_sensor_read(&slot, &actual, NULL) == HBALL_IPC_HEADER);

    memset(&slot, 0, sizeof(slot));
    slot.bytes[0] = 1U;
    assert(hball_ipc_sensor_read(&slot, &actual, NULL) == HBALL_IPC_BUSY);

    assert(hball_ipc_sensor_publish(&slot, &expected, NULL) == HBALL_IPC_OK);
    probe.tear_on_second_invalidate = true;
    assert(hball_ipc_sensor_read(&slot, &actual, &ops) == HBALL_IPC_BUSY);
}

static void test_nonfinite_or_out_of_range_values_are_rejected(void)
{
    hball_ipc_sensor_slot_t sensor_slot = {0};
    hball_ipc_control_slot_t control_slot = {0};
    hball_sensor_snapshot_t sensor = make_sensor_snapshot();
    hball_control_shadow_t control = {0};

    sensor.vision_confidence = 1.1F;
    assert(hball_ipc_sensor_publish(&sensor_slot, &sensor, NULL)
        == HBALL_IPC_RANGE);
    sensor = make_sensor_snapshot();
    sensor.motor_angle_rad = NAN;
    assert(hball_ipc_sensor_publish(&sensor_slot, &sensor, NULL)
        == HBALL_IPC_RANGE);

    control.flags = HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY;
    control.target_angle_rad = INFINITY;
    assert(hball_ipc_control_publish(&control_slot, &control, NULL)
        == HBALL_IPC_RANGE);
    memset(&control, 0, sizeof(control));
    assert(hball_ipc_control_publish(&control_slot, &control, NULL)
        == HBALL_IPC_RANGE);
}

int main(void)
{
    test_layout_and_crc();
    test_sensor_round_trip_and_cache_contract();
    test_control_round_trip();
    test_corruption_busy_and_torn_read_fail_closed();
    test_nonfinite_or_out_of_range_values_are_rejected();
    return 0;
}
