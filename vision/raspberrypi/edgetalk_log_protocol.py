"""Decoder for CRC-protected EdgeTalk HBLG USB telemetry."""

from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = b"HBLG"
V1_FRAME_SIZE = 80
V2_FRAME_SIZE = 288
VERSION_1 = 1
VERSION_2 = 2
STATUS_Q3_ACTUAL = 1 << 16
STATUS_CONTROL_ACTIVE = 1 << 17
STATUS_Q3_PASSED = 1 << 18
STATUS_ACTUAL_CONTROL = 1 << 19

V1_VALUE_NAMES = (
    "ball_position_m",
    "estimated_position_m",
    "estimated_velocity_mps",
    "estimated_disturbance_mps2",
    "pipe_target_rad",
    "motor_angle_rad",
    "motor_velocity_rad_s",
    "longitudinal_accel_mps2",
    "body_pitch_rad",
)

V2_VALUE_NAMES = (
    "ball_position_m",
    "vision_confidence",
    "estimated_position_m",
    "estimated_velocity_mps",
    "target_position_m",
    "estimated_disturbance_mps2",
    "pipe_target_rad",
    "actual_pipe_angle_rad",
    "motor_target_rad",
    "motor_angle_rad",
    "motor_velocity_rad_s",
    "motor_torque_nm",
    "motor_temperature_c",
    "motor_filtered_iq_a",
    "motor_vbus_v",
    "accel_x_mps2",
    "accel_y_mps2",
    "accel_z_mps2",
    "gyro_x_rad_s",
    "gyro_y_rad_s",
    "gyro_z_rad_s",
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "wheel_left_mps",
    "wheel_right_mps",
    "body_speed_mps",
    "controller_position_error_m",
    "controller_integral_error_m_s",
    "controller_filtered_accel_mps2",
    "controller_feedback_rad",
    "controller_feedforward_rad",
    "controller_requested_rad",
    "controller_command_rad",
    "controller_p_rad",
    "controller_i_rad",
    "controller_d_rad",
    "controller_command_limit_rad",
    "controller_rate_limit_rad_s",
)

# New consumers should use the V2 superset. ControlLog.value_names identifies
# the actual tuple layout of each decoded frame.
VALUE_NAMES = V2_VALUE_NAMES

V2_METADATA_NAMES = (
    "vision_capture_time_us",
    "vision_receive_time_ms",
    "vision_processing_time_us",
    "imu_epoch",
    "imu_sample_mask",
    "imu_source_time_ms",
    "accel_receive_time_ms",
    "gyro_receive_time_ms",
    "attitude_receive_time_ms",
    "imu_sync_receive_time_ms",
    "wheel_sequence",
    "accel_sequence",
    "gyro_sequence",
    "attitude_sequence",
    "wheel_receive_time_ms",
    "motor_receive_time_ms",
    "msp_status_flags",
    "vision_flags",
    "motor_fault_summary",
    "motor_mode_state",
    "motor_run_mode",
    "control_output_flags",
    "vision_age_ms",
    "imu_age_ms",
    "wheel_age_ms",
    "motor_age_ms",
    "heartbeat_age_ms",
)

_V1_FORMAT = struct.Struct("<4sHH6IHHI9fI")
_V2_PREFIX_FORMAT = struct.Struct("<4sHH6IHHIQIIHH5I4HIIHH4B6I")
_V2_VALUES_FORMAT = struct.Struct("<39f")


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return (~crc) & 0xFFFFFFFF


@dataclass(frozen=True)
class ControlLog:
    version: int
    frame_size: int
    sequence: int
    produced_time_ms: int
    sensor_sequence: int
    controller_steps: int
    vision_sequence: int
    sensor_valid_flags: int
    control_mode: int
    guard_reason: int
    status_flags: int
    metadata: dict[str, int]
    value_names: tuple[str, ...]
    values: tuple[float, ...]
    raw: bytes

    @property
    def is_q3_actual(self) -> bool:
        return bool(self.status_flags & STATUS_Q3_ACTUAL)

    @property
    def is_actual_control(self) -> bool:
        return bool(self.status_flags & STATUS_ACTUAL_CONTROL) or self.is_q3_actual

    def named_values(self) -> dict[str, float]:
        values = dict(zip(self.value_names, self.values, strict=True))
        if self.version == VERSION_1 and self.is_q3_actual:
            values["target_position_m"] = values["estimated_disturbance_mps2"]
        return values


def _decode_v1(frame: bytes) -> ControlLog:
    fields = _V1_FORMAT.unpack(frame)
    return ControlLog(
        version=VERSION_1,
        frame_size=V1_FRAME_SIZE,
        sequence=fields[3],
        produced_time_ms=fields[4],
        sensor_sequence=fields[5],
        controller_steps=fields[6],
        vision_sequence=fields[7],
        sensor_valid_flags=fields[8],
        control_mode=fields[9],
        guard_reason=fields[10],
        status_flags=fields[11],
        metadata={},
        value_names=V1_VALUE_NAMES,
        values=tuple(fields[12:21]),
        raw=frame,
    )


def _decode_v2(frame: bytes) -> ControlLog:
    fields = _V2_PREFIX_FORMAT.unpack(frame[: _V2_PREFIX_FORMAT.size])
    metadata_values = (
        fields[12], fields[13], fields[14], fields[15], fields[16],
        fields[17], fields[18], fields[19], fields[20], fields[21],
        fields[22], fields[23], fields[24], fields[25], fields[26],
        fields[27], fields[28], fields[29], fields[30], fields[31],
        fields[32], fields[34], fields[35], fields[36], fields[37],
        fields[38], fields[39],
    )
    return ControlLog(
        version=VERSION_2,
        frame_size=V2_FRAME_SIZE,
        sequence=fields[3],
        produced_time_ms=fields[4],
        sensor_sequence=fields[5],
        controller_steps=fields[6],
        status_flags=fields[7],
        sensor_valid_flags=fields[8],
        control_mode=fields[9],
        guard_reason=fields[10],
        vision_sequence=fields[11],
        metadata=dict(zip(V2_METADATA_NAMES, metadata_values, strict=True)),
        value_names=V2_VALUE_NAMES,
        values=_V2_VALUES_FORMAT.unpack(frame[128:-4]),
        raw=frame,
    )


def decode_frame(frame: bytes) -> ControlLog:
    if len(frame) < 8:
        raise ValueError("control-log frame is shorter than its header")
    magic, version, frame_size = struct.unpack_from("<4sHH", frame)
    expected_size = {
        VERSION_1: V1_FRAME_SIZE,
        VERSION_2: V2_FRAME_SIZE,
    }.get(version)
    if magic != MAGIC or expected_size is None or frame_size != expected_size:
        raise ValueError("wrong control-log header")
    if len(frame) != expected_size:
        raise ValueError("wrong control-log frame length")
    expected_crc = struct.unpack_from("<I", frame, len(frame) - 4)[0]
    if crc32c(frame[:-4]) != expected_crc:
        raise ValueError("wrong control-log CRC32C")
    return _decode_v1(frame) if version == VERSION_1 else _decode_v2(frame)


class ControlLogStream:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_failures = 0
        self.discarded_bytes = 0

    def push(self, data: bytes) -> list[ControlLog]:
        self.buffer.extend(data)
        decoded: list[ControlLog] = []
        while True:
            offset = self.buffer.find(MAGIC)
            if offset < 0:
                keep = min(len(self.buffer), len(MAGIC) - 1)
                self.discarded_bytes += len(self.buffer) - keep
                del self.buffer[: len(self.buffer) - keep]
                break
            if offset:
                self.discarded_bytes += offset
                del self.buffer[:offset]
            if len(self.buffer) < 8:
                break
            _, version, frame_size = struct.unpack_from("<4sHH", self.buffer)
            expected_size = {
                VERSION_1: V1_FRAME_SIZE,
                VERSION_2: V2_FRAME_SIZE,
            }.get(version)
            if expected_size is None or frame_size != expected_size:
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < expected_size:
                break
            candidate = bytes(self.buffer[:expected_size])
            try:
                decoded.append(decode_frame(candidate))
                del self.buffer[:expected_size]
            except ValueError:
                self.crc_failures += 1
                self.discarded_bytes += 1
                del self.buffer[0]
        return decoded
