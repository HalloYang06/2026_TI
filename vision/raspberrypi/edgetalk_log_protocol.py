"""Decoder for the read-only HBALL_CONTROL_LOG_V1 USB stream."""

from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = b"HBLG"
FRAME_SIZE = 80
VERSION = 1
STATUS_Q3_ACTUAL = 1 << 16
STATUS_CONTROL_ACTIVE = 1 << 17
STATUS_Q3_PASSED = 1 << 18
VALUE_NAMES = (
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
_FORMAT = struct.Struct("<4sHHIIIIIIHHI9fI")


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return (~crc) & 0xFFFFFFFF


@dataclass(frozen=True)
class ControlLog:
    sequence: int
    produced_time_ms: int
    sensor_sequence: int
    controller_steps: int
    vision_sequence: int
    sensor_valid_flags: int
    control_mode: int
    guard_reason: int
    status_flags: int
    values: tuple[float, ...]
    raw: bytes

    @property
    def is_q3_actual(self) -> bool:
        return bool(self.status_flags & STATUS_Q3_ACTUAL)

    def named_values(self) -> dict[str, float]:
        values = dict(zip(VALUE_NAMES, self.values, strict=True))
        if self.is_q3_actual:
            values["target_position_m"] = values["estimated_disturbance_mps2"]
        return values


def decode_frame(frame: bytes) -> ControlLog:
    if len(frame) != FRAME_SIZE:
        raise ValueError("wrong control-log frame length")
    fields = _FORMAT.unpack(frame)
    if fields[0] != MAGIC or fields[1] != VERSION or fields[2] != FRAME_SIZE:
        raise ValueError("wrong control-log header")
    if crc32c(frame[:76]) != fields[-1]:
        raise ValueError("wrong control-log CRC32C")
    return ControlLog(
        sequence=fields[3], produced_time_ms=fields[4],
        sensor_sequence=fields[5], controller_steps=fields[6],
        vision_sequence=fields[7], sensor_valid_flags=fields[8],
        control_mode=fields[9], guard_reason=fields[10],
        status_flags=fields[11], values=tuple(fields[12:21]), raw=frame,
    )


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
            if len(self.buffer) < FRAME_SIZE:
                break
            candidate = bytes(self.buffer[:FRAME_SIZE])
            try:
                decoded.append(decode_frame(candidate))
                del self.buffer[:FRAME_SIZE]
            except ValueError:
                self.crc_failures += 1
                self.discarded_bytes += 1
                del self.buffer[0]
        return decoded
