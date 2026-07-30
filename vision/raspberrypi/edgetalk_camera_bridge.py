#!/usr/bin/env python3
"""Forward real C++ camera measurements to EdgeTalk over USB CDC.

This program never sends CAN or actuator commands.  It forwards only new
measurement records from the local ball-camera service as VISION_MEASUREMENT_V1
frames.  When the camera cannot find a ball, it sends a heartbeat with
POSITION_VALID cleared.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from typing import Any
from urllib.request import urlopen

from edgetalk_usb_daemon import SingleInstanceLock, find_serial_port
from edgetalk_log_protocol import ControlLogStream
from vision_measurement_protocol import (
    FLAG_DETECTED,
    FLAG_POSITION_VALID,
    VisionMeasurement,
    encode_measurement,
)


WRITE_TIMEOUT_S = 0.020


def _finite_number(payload: dict[str, Any], name: str, default: float = 0.0) -> float:
    value = payload.get(name, default)
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        return default
    return float(value)


def _uint(payload: dict[str, Any], name: str, maximum: int) -> int:
    value = payload.get(name, 0)
    return int(value) if isinstance(value, int) and 0 <= value <= maximum else 0


def build_measurement(payload: dict[str, Any]) -> VisionMeasurement:
    """Translate one /data SSE JSON object to the shared binary protocol."""
    found = payload.get("found") is True
    flags = 0
    if found:
        flags |= (FLAG_DETECTED | FLAG_POSITION_VALID)
    return VisionMeasurement(
        flags=flags,
        sequence=_uint(payload, "sequence", 0xFFFFFFFF),
        capture_time_us=_uint(payload, "capture_time_us", 0xFFFFFFFFFFFFFFFF),
        processing_time_us=_uint(payload, "processing_time_us", 0xFFFFFFFF),
        center_x_px=_finite_number(payload, "center_x_px"),
        center_y_px=_finite_number(payload, "center_y_px"),
        radius_px=max(0.0, _finite_number(payload, "radius_px")),
        ball_position_m=(_finite_number(payload, "position_cm") / 100.0) if found else 0.0,
        confidence=0.85 if found else 0.0,
        contour_area_px2=max(0.0, _finite_number(payload, "contour_area_px2")),
        roi_x=_uint(payload, "roi_x", 0xFFFF),
        roi_y=_uint(payload, "roi_y", 0xFFFF),
        roi_w=_uint(payload, "roi_w", 0xFFFF),
        roi_h=_uint(payload, "roi_h", 0xFFFF),
        exposure_us=0,
    )


def forward_stream(
    source_url: str, device, *, max_frames: int | None = None,
    telemetry_log=None,
) -> int:
    """Forward fresh SSE records; a disconnect raises so the caller reconnects."""
    forwarded = 0
    last_sequence: int | None = None
    telemetry = ControlLogStream()
    with urlopen(source_url, timeout=5.0) as response:
        for raw_line in response:
            if not raw_line.startswith(b"data: "):
                continue
            try:
                payload = json.loads(raw_line[6:].decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(payload, dict):
                continue
            sequence = _uint(payload, "sequence", 0xFFFFFFFF)
            if last_sequence == sequence:
                continue
            frame = encode_measurement(build_measurement(payload))
            written = device.write(frame)
            if written != len(frame):
                raise OSError(f"short USB CDC write: {written}/{len(frame)}")
            device.flush()
            waiting = getattr(device, "in_waiting", 0)
            if waiting:
                for record in telemetry.push(device.read(waiting)):
                    if telemetry_log is not None:
                        telemetry_log.write(record.raw)
            last_sequence = sequence
            forwarded += 1
            if max_frames is not None and forwarded >= max_frames:
                return forwarded
    raise OSError("camera SSE stream ended")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-url", default="http://127.0.0.1:8080/data")
    parser.add_argument("--port", help="EdgeTalk /dev/serial/by-id path; auto-detect by default")
    parser.add_argument("--retry", type=float, default=1.0)
    parser.add_argument("--lock-file", default="/tmp/hball-edgetalk-camera.lock")
    parser.add_argument(
        "--telemetry-log",
        help="append validated 80-byte control records; disabled by default",
    )
    arguments = parser.parse_args()
    if arguments.retry <= 0.0:
        parser.error("retry must be positive")

    try:
        import serial
    except ImportError:
        print("FAIL: install python3-serial", file=sys.stderr)
        return 2

    lock = SingleInstanceLock(arguments.lock_file)
    if not lock.acquire():
        print("FAIL: another EdgeTalk camera sender owns the USB link", file=sys.stderr)
        return 3
    telemetry_log = None
    try:
        telemetry_log = (
            open(arguments.telemetry_log, "ab", buffering=0)
            if arguments.telemetry_log else None
        )
        while True:
            try:
                port = find_serial_port(arguments.port)
                with serial.Serial(port=port, baudrate=115200, timeout=0.0,
                                   write_timeout=WRITE_TIMEOUT_S, exclusive=True) as device:
                    device.reset_input_buffer()
                    count = forward_stream(
                        arguments.source_url, device,
                        telemetry_log=telemetry_log,
                    )
                    print(f"RECONNECT: SSE ended after {count} frames", file=sys.stderr)
            except (OSError, RuntimeError, ValueError) as error:
                print(f"RECONNECT: {error}", file=sys.stderr)
            time.sleep(arguments.retry)
    except KeyboardInterrupt:
        return 0
    finally:
        if telemetry_log is not None:
            telemetry_log.close()
        lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
