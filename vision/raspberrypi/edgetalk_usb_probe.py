#!/usr/bin/env python3
"""Read-only USB CDC heartbeat and echo probe for the H-ball EdgeTalk firmware."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import Optional


PROTOCOL_VERSION = "0.1.0"
MAX_PAYLOAD = 48
UINT32_MAX = (1 << 32) - 1


def _decode_line(line: bytes) -> Optional[str]:
    try:
        decoded = line.decode("ascii", errors="strict").rstrip("\r\n")
    except UnicodeDecodeError:
        return None
    if not decoded or any(ord(character) < 0x20 or ord(character) > 0x7E for character in decoded):
        return None
    return decoded


def _parse_uint32(value: str) -> Optional[int]:
    if not value.isdecimal():
        return None
    parsed = int(value, 10)
    return parsed if 0 <= parsed <= UINT32_MAX else None


def build_ping(sequence: int, payload: str = "") -> bytes:
    if not 0 <= sequence <= UINT32_MAX:
        raise ValueError("sequence must fit uint32")
    if len(payload) > MAX_PAYLOAD or any(
        ord(character) < 0x20 or ord(character) > 0x7E for character in payload
    ):
        raise ValueError("payload must be at most 48 printable ASCII characters")
    suffix = f" {payload}" if payload else ""
    return f"PING {sequence}{suffix}\n".encode("ascii")


def parse_pong(line: bytes) -> Optional[tuple[int, str]]:
    decoded = _decode_line(line)
    if decoded is None:
        return None
    fields = decoded.split(" ", 2)
    if len(fields) < 2 or fields[0] != "PONG":
        return None
    sequence = _parse_uint32(fields[1])
    payload = fields[2] if len(fields) == 3 else ""
    if sequence is None or len(payload) > MAX_PAYLOAD:
        return None
    return sequence, payload


def parse_ready(line: bytes) -> Optional[tuple[int, int]]:
    decoded = _decode_line(line)
    if decoded is None:
        return None
    fields = decoded.split(" ")
    if len(fields) != 4 or fields[:2] != ["HBALL_USB_READY", PROTOCOL_VERSION]:
        return None
    sequence = _parse_uint32(fields[2])
    uptime_ms = _parse_uint32(fields[3])
    if sequence is None or uptime_ms is None:
        return None
    return sequence, uptime_ms


def find_serial_port(explicit_port: Optional[str]) -> str:
    if explicit_port:
        return explicit_port

    by_id = Path("/dev/serial/by-id")
    preferred = sorted(
        str(path) for path in by_id.glob("*") if "HBall" in path.name
    ) if by_id.exists() else []
    candidates = preferred or sorted(str(path) for path in Path("/dev").glob("ttyACM*"))
    if len(candidates) != 1:
        listing = ", ".join(candidates) if candidates else "none"
        raise RuntimeError(
            "expected exactly one EdgeTalk CDC port; "
            f"found {len(candidates)} ({listing}); pass --port explicitly"
        )
    return candidates[0]


def _wait_for_ready(serial_port, timeout_s: float) -> tuple[int, int]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        ready = parse_ready(serial_port.readline())
        if ready is not None:
            return ready
    raise TimeoutError("no HBALL_USB_READY heartbeat received")


def _synchronize_link(serial_port, timeout_s: float) -> tuple[int, int]:
    serial_port.reset_input_buffer()
    serial_port.write(b"\n")
    serial_port.flush()
    return _wait_for_ready(serial_port, timeout_s)


def run_probe(port: str, duration_s: float, rate_hz: float, timeout_s: float) -> dict[str, object]:
    try:
        import serial
    except ImportError as error:
        raise RuntimeError(
            "pyserial is required; install the Raspberry Pi package python3-serial"
        ) from error

    period_s = 1.0 / rate_hz
    latencies_ms: list[float] = []
    counters = {"tx": 0, "rx": 0, "timeout": 0, "unexpected": 0}

    with serial.Serial(
        port=port,
        baudrate=115200,
        timeout=min(0.02, timeout_s),
        write_timeout=1.0,
    ) as device:
        ready_sequence, ready_uptime_ms = _synchronize_link(device, 5.0)
        deadline = time.monotonic() + duration_s
        sequence = 0

        while time.monotonic() < deadline:
            cycle_start = time.monotonic()
            payload = f"host_ms={int(cycle_start * 1000.0) & UINT32_MAX}"
            device.write(build_ping(sequence, payload))
            device.flush()
            counters["tx"] += 1

            reply_deadline = cycle_start + timeout_s
            matched = False
            while time.monotonic() < reply_deadline:
                line = device.readline()
                if not line or parse_ready(line) is not None:
                    continue
                pong = parse_pong(line)
                if pong == (sequence, payload):
                    counters["rx"] += 1
                    latencies_ms.append((time.monotonic() - cycle_start) * 1000.0)
                    matched = True
                    break
                counters["unexpected"] += 1
            if not matched:
                counters["timeout"] += 1

            sequence = (sequence + 1) & UINT32_MAX
            remaining = period_s - (time.monotonic() - cycle_start)
            if remaining > 0.0:
                time.sleep(remaining)

    ordered = sorted(latencies_ms)
    p95_ms = ordered[max(0, ((len(ordered) * 95 + 99) // 100) - 1)] if ordered else None
    return {
        **counters,
        "ready_sequence": ready_sequence,
        "ready_uptime_ms": ready_uptime_ms,
        "mean_rtt_ms": (sum(ordered) / len(ordered)) if ordered else None,
        "p95_rtt_ms": p95_ms,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="CDC port; auto-detects one /dev/ttyACM* by default")
    parser.add_argument("--duration", type=float, default=60.0, help="test duration in seconds")
    parser.add_argument("--rate", type=float, default=20.0, help="PING rate in Hz")
    parser.add_argument("--timeout", type=float, default=0.25, help="per-PING timeout in seconds")
    arguments = parser.parse_args()
    if arguments.duration <= 0.0 or arguments.rate <= 0.0 or arguments.timeout <= 0.0:
        parser.error("duration, rate and timeout must be positive")

    try:
        port = find_serial_port(arguments.port)
        result = run_probe(port, arguments.duration, arguments.rate, arguments.timeout)
    except (OSError, RuntimeError, TimeoutError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 2

    print(f"port={port}")
    print(" ".join(f"{key}={value}" for key, value in result.items()))
    passed = (
        result["tx"] > 0
        and result["rx"] == result["tx"]
        and result["timeout"] == 0
        and result["unexpected"] == 0
    )
    print("PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
