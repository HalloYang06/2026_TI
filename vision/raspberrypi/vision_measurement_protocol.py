"""Binary 64-byte Raspberry Pi to EdgeTalk ball measurement protocol."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


MAGIC = 0x5AA5
VERSION = 1
MESSAGE_TYPE = 1
FRAME_SIZE = 64
MAGIC_BYTES = struct.pack("<H", MAGIC)
_HEAD = struct.Struct("<HBBHHIQIffffffHHHHI")
_FRAME = struct.Struct("<HBBHHIQIffffffHHHHII")

FLAG_DETECTED = 1 << 0
FLAG_POSITION_VALID = 1 << 1
FLAG_ROI_PREDICTED = 1 << 2
FLAG_ROI_CLIPPED = 1 << 3
FLAG_EXPOSURE_STABLE = 1 << 4
FLAG_CONTOUR_ROUND = 1 << 5


class FrameError(ValueError):
    def __init__(self, reason: str):
        super().__init__(f"invalid vision frame: {reason}")
        self.reason = reason


@dataclass
class VisionMeasurement:
    flags: int
    sequence: int
    capture_time_us: int
    processing_time_us: int
    center_x_px: float
    center_y_px: float
    radius_px: float
    ball_position_m: float
    confidence: float
    contour_area_px2: float
    roi_x: int
    roi_y: int
    roi_w: int
    roi_h: int
    exposure_us: int


def crc32c(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def _validate(measurement: VisionMeasurement) -> None:
    integer_ranges = (
        (measurement.flags, 0xFFFF),
        (measurement.sequence, 0xFFFFFFFF),
        (measurement.capture_time_us, 0xFFFFFFFFFFFFFFFF),
        (measurement.processing_time_us, 0xFFFFFFFF),
        (measurement.roi_x, 0xFFFF),
        (measurement.roi_y, 0xFFFF),
        (measurement.roi_w, 0xFFFF),
        (measurement.roi_h, 0xFFFF),
        (measurement.exposure_us, 0xFFFFFFFF),
    )
    if any(not isinstance(value, int) or not 0 <= value <= maximum
           for value, maximum in integer_ranges):
        raise ValueError("integer field is outside its wire range")

    floats = (
        measurement.center_x_px,
        measurement.center_y_px,
        measurement.radius_px,
        measurement.ball_position_m,
        measurement.confidence,
        measurement.contour_area_px2,
    )
    if not all(math.isfinite(value) for value in floats):
        raise ValueError("floating-point fields must be finite")
    if measurement.radius_px < 0.0 or measurement.contour_area_px2 < 0.0:
        raise ValueError("radius and contour area must be non-negative")
    if not 0.0 <= measurement.confidence <= 1.0:
        raise ValueError("confidence must be in [0, 1]")


def encode_measurement(measurement: VisionMeasurement) -> bytes:
    _validate(measurement)
    head = _HEAD.pack(
        MAGIC, VERSION, MESSAGE_TYPE, FRAME_SIZE,
        measurement.flags, measurement.sequence,
        measurement.capture_time_us, measurement.processing_time_us,
        measurement.center_x_px, measurement.center_y_px,
        measurement.radius_px, measurement.ball_position_m,
        measurement.confidence, measurement.contour_area_px2,
        measurement.roi_x, measurement.roi_y,
        measurement.roi_w, measurement.roi_h, measurement.exposure_us,
    )
    return head + struct.pack("<I", crc32c(head))


def decode_measurement(frame: bytes) -> VisionMeasurement:
    if len(frame) != FRAME_SIZE:
        raise FrameError("size")
    fields = _FRAME.unpack(frame)
    magic, version, message_type, frame_size = fields[:4]
    if magic != MAGIC:
        raise FrameError("magic")
    if (version, message_type, frame_size) != (VERSION, MESSAGE_TYPE, FRAME_SIZE):
        raise FrameError("header")
    if crc32c(frame[:-4]) != fields[-1]:
        raise FrameError("crc")
    measurement = VisionMeasurement(*fields[4:-1])
    try:
        _validate(measurement)
    except ValueError as error:
        raise FrameError("range") from error
    return measurement


class VisionStreamDecoder:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.frames_accepted = 0
        self.crc_failures = 0
        self.header_failures = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[VisionMeasurement]:
        self._buffer.extend(data)
        decoded: list[VisionMeasurement] = []
        while True:
            magic_at = self._buffer.find(MAGIC_BYTES)
            if magic_at < 0:
                keep = 1 if self._buffer[-1:] == MAGIC_BYTES[:1] else 0
                self.discarded_bytes += len(self._buffer) - keep
                del self._buffer[: len(self._buffer) - keep]
                break
            if magic_at:
                self.discarded_bytes += magic_at
                del self._buffer[:magic_at]
            if len(self._buffer) < FRAME_SIZE:
                break
            try:
                measurement = decode_measurement(bytes(self._buffer[:FRAME_SIZE]))
            except FrameError as error:
                if error.reason == "crc":
                    self.crc_failures += 1
                else:
                    self.header_failures += 1
                del self._buffer[0]
                continue
            decoded.append(measurement)
            self.frames_accepted += 1
            del self._buffer[:FRAME_SIZE]
        return decoded
