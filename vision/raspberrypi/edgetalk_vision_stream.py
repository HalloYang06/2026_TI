#!/usr/bin/env python3
"""Send synthetic 64-byte vision frames to an unpowered EdgeTalk bench."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Callable

from edgetalk_usb_probe import find_serial_port
from vision_measurement_protocol import (
    FLAG_CONTOUR_ROUND,
    FLAG_DETECTED,
    FLAG_EXPOSURE_STABLE,
    FLAG_POSITION_VALID,
    VisionMeasurement,
    decode_measurement,
    encode_measurement,
)


def build_synthetic_measurement(sequence: int, capture_time_us: int) -> VisionMeasurement:
    phase = (sequence % 120) / 120.0 * 2.0 * math.pi
    position_m = 0.04 * math.sin(phase)
    return VisionMeasurement(
        flags=(FLAG_DETECTED | FLAG_POSITION_VALID
               | FLAG_EXPOSURE_STABLE | FLAG_CONTOUR_ROUND),
        sequence=sequence,
        capture_time_us=capture_time_us,
        processing_time_us=1500,
        center_x_px=160.0 + position_m * 1000.0,
        center_y_px=60.0,
        radius_px=6.0,
        ball_position_m=position_m,
        confidence=0.95,
        contour_area_px2=113.0,
        roi_x=0,
        roi_y=0,
        roi_w=320,
        roi_h=120,
        exposure_us=2000,
    )


def _percentile_95(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, ((len(ordered) * 95 + 99) // 100) - 1)]


def rate_passes(result: dict[str, int | float | None], minimum_rate_hz: float) -> bool:
    achieved = result.get("achieved_rate_hz")
    misses = result.get("deadline_misses")
    return (
        isinstance(achieved, (int, float))
        and achieved >= minimum_rate_hz
        and misses == 0
    )


def run_stream(
    serial_port,
    duration_s: float,
    rate_hz: float,
    *,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, int | float | None]:
    if duration_s <= 0.0 or rate_hz <= 0.0:
        raise ValueError("duration and rate must be positive")

    period_s = 1.0 / rate_hz
    started_at = monotonic()
    slot_count = max(1, math.ceil(duration_s * rate_hz - 1.0e-12))
    slot = 0
    sequence = 0
    tx_frames = 0
    deadline_misses = 0
    schedule_lag_ms: list[float] = []
    write_time_ms: list[float] = []
    write_finished = started_at

    while slot < slot_count:
        scheduled_at = started_at + slot * period_s
        now = monotonic()
        if now < scheduled_at:
            sleep(scheduled_at - now)
        write_started = monotonic()
        schedule_lag_ms.append(max(0.0, write_started - scheduled_at) * 1000.0)
        frame = encode_measurement(
            build_synthetic_measurement(sequence, int(write_started * 1_000_000.0))
        )
        written = serial_port.write(frame)
        write_finished = monotonic()
        if written != len(frame):
            raise OSError(f"short USB CDC write: {written}/{len(frame)}")
        write_time_ms.append((write_finished - write_started) * 1000.0)
        sequence = (sequence + 1) & 0xFFFFFFFF
        tx_frames += 1

        slot += 1
        while (slot < slot_count
               and (started_at + slot * period_s) < write_finished):
            deadline_misses += 1
            slot += 1

    serial_port.flush()
    elapsed_s = max(duration_s, write_finished - started_at)
    achieved_rate_hz = tx_frames / elapsed_s
    return {
        "tx_frames": tx_frames,
        "tx_bytes": tx_frames * 64,
        "deadline_misses": deadline_misses,
        "achieved_rate_hz": achieved_rate_hz,
        "payload_rate_bytes_s": achieved_rate_hz * 64.0,
        "mean_schedule_lag_ms": (
            sum(schedule_lag_ms) / len(schedule_lag_ms) if schedule_lag_ms else None
        ),
        "p95_schedule_lag_ms": _percentile_95(schedule_lag_ms),
        "p95_write_ms": _percentile_95(write_time_ms),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="CDC port; auto-detects /dev/ttyACM* by default")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--rate", type=float, default=240.0)
    parser.add_argument(
        "--min-rate",
        type=float,
        help="minimum achieved frame rate; defaults to 99%% of --rate",
    )
    arguments = parser.parse_args()
    if arguments.duration <= 0.0 or arguments.rate <= 0.0:
        parser.error("duration and rate must be positive")
    minimum_rate_hz = (
        arguments.min_rate if arguments.min_rate is not None else arguments.rate * 0.99
    )
    if minimum_rate_hz <= 0.0:
        parser.error("min-rate must be positive")

    try:
        import serial
    except ImportError:
        print("FAIL: install python3-serial", file=sys.stderr)
        return 2

    try:
        port = find_serial_port(arguments.port)
        with serial.Serial(
            port=port, baudrate=115200, timeout=0.0, write_timeout=1.0
        ) as device:
            device.reset_input_buffer()
            result = run_stream(device, arguments.duration, arguments.rate)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 2

    print(f"port={port}")
    print("payload=BALL_MEASUREMENT_V1 frame_bytes=64 image_bytes=0")
    print(" ".join(f"{key}={value}" for key, value in result.items()))
    if not rate_passes(result, minimum_rate_hz):
        print(
            f"HOST_FAIL: min_rate_hz={minimum_rate_hz}; "
            "confirm CPU scheduling and USB backpressure",
            file=sys.stderr,
        )
        return 2
    print(
        f"HOST_PASS min_rate_hz={minimum_rate_hz}; "
        "confirm M33 vision_rate_x10 and CRC counters with hball_usb_status"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
