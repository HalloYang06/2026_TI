#!/usr/bin/env python3
"""Keep a fail-safe PING-only USB CDC link to EdgeTalk across reconnects."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import time
from typing import Callable, Optional

from edgetalk_usb_probe import build_ping, parse_pong, parse_ready


DEFAULT_BAUDRATE = 115200
DEFAULT_RETRY_S = 1.0
DEFAULT_PING_RATE_HZ = 2.0
DEFAULT_READY_TIMEOUT_S = 5.0
DEFAULT_PING_TIMEOUT_S = 0.5


def find_serial_port(
    explicit_port: Optional[str],
    *,
    by_id_dir: Path = Path("/dev/serial/by-id"),
    dev_dir: Path = Path("/dev"),
) -> str:
    """Select one stable by-id path, falling back to one unambiguous ttyACM."""
    if explicit_port:
        return explicit_port

    stable_ports = sorted(
        (path for path in by_id_dir.glob("*") if path.exists() or path.is_symlink()),
        key=lambda path: path.name,
    ) if by_id_dir.exists() else []
    named_hball = [path for path in stable_ports if "hball" in path.name.casefold()]
    candidates = named_hball or stable_ports
    if len(candidates) == 1:
        return str(candidates[0])
    if len(candidates) > 1:
        raise RuntimeError(
            "multiple stable serial ports found; set --port to one /dev/serial/by-id path"
        )

    fallback = sorted(dev_dir.glob("ttyACM*"), key=lambda path: path.name)
    if len(fallback) == 1:
        return str(fallback[0])
    raise RuntimeError(
        f"expected one EdgeTalk CDC port; found {len(fallback)} ttyACM candidates"
    )


def open_serial(port: str, *, serial_factory=None):
    if serial_factory is None:
        try:
            import serial
        except ImportError as error:
            raise RuntimeError("install the Raspberry Pi package python3-serial") from error
        serial_factory = serial.Serial
    return serial_factory(
        port=port,
        baudrate=DEFAULT_BAUDRATE,
        timeout=0.05,
        write_timeout=0.5,
        exclusive=True,
    )


def _wait_for_ready(serial_port, timeout_s: float, monotonic) -> tuple[int, int]:
    deadline = monotonic() + timeout_s
    while monotonic() < deadline:
        ready = parse_ready(serial_port.readline())
        if ready is not None:
            return ready
    raise TimeoutError("no HBALL_USB_READY heartbeat received")


def run_safe_ping_session(
    serial_port,
    *,
    ready_timeout_s: float = DEFAULT_READY_TIMEOUT_S,
    ping_timeout_s: float = DEFAULT_PING_TIMEOUT_S,
    ping_rate_hz: float = DEFAULT_PING_RATE_HZ,
    max_pings: Optional[int] = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, int]:
    """Synchronize with READY, then emit PING only; never emit vision frames."""
    if ready_timeout_s <= 0.0 or ping_timeout_s <= 0.0 or ping_rate_hz <= 0.0:
        raise ValueError("timeouts and ping rate must be positive")

    serial_port.reset_input_buffer()
    serial_port.write(b"\n")
    serial_port.flush()
    ready_sequence, ready_uptime_ms = _wait_for_ready(
        serial_port, ready_timeout_s, monotonic
    )
    print(f"READY: sequence={ready_sequence} uptime_ms={ready_uptime_ms}")

    sequence = 0
    pings = 0
    period_s = 1.0 / ping_rate_hz
    while max_pings is None or pings < max_pings:
        cycle_started = monotonic()
        payload = "startup_safe=1"
        request = build_ping(sequence, payload)
        if serial_port.write(request) != len(request):
            raise OSError("short USB CDC PING write")
        serial_port.flush()

        reply_deadline = monotonic() + ping_timeout_s
        while monotonic() < reply_deadline:
            line = serial_port.readline()
            if parse_ready(line) is not None:
                continue
            if parse_pong(line) == (sequence, payload):
                break
        else:
            raise TimeoutError(f"PING {sequence} timed out")

        pings += 1
        sequence = (sequence + 1) & 0xFFFFFFFF
        remaining = period_s - (monotonic() - cycle_started)
        if remaining > 0.0:
            sleep(remaining)

    return {
        "ready_sequence": ready_sequence,
        "ready_uptime_ms": ready_uptime_ms,
        "pings": pings,
    }


def supervise_link(
    resolve_port,
    run_session,
    *,
    retry_s: float = DEFAULT_RETRY_S,
    sleep: Callable[[float], None] = time.sleep,
    max_cycles: Optional[int] = None,
) -> dict[str, int]:
    """Wait for a device and restart the safe session after every disconnect."""
    if retry_s <= 0.0:
        raise ValueError("retry interval must be positive")
    stats = {
        "cycles": 0,
        "device_waits": 0,
        "connections": 0,
        "disconnects": 0,
        "sessions_completed": 0,
    }
    while max_cycles is None or stats["cycles"] < max_cycles:
        stats["cycles"] += 1
        try:
            port = resolve_port()
        except RuntimeError as error:
            stats["device_waits"] += 1
            print(f"WAIT: {error}", file=sys.stderr)
        else:
            stats["connections"] += 1
            try:
                run_session(port)
            except (OSError, RuntimeError, TimeoutError, ValueError) as error:
                stats["disconnects"] += 1
                print(f"RECONNECT: {error}", file=sys.stderr)
            else:
                stats["sessions_completed"] += 1
        sleep(retry_s)
    return stats


class SingleInstanceLock:
    def __init__(self, path: Path, *, flock_module=None) -> None:
        self.path = Path(path)
        self._flock_module = flock_module
        self._handle = None

    def acquire(self) -> bool:
        if self._flock_module is None:
            try:
                import fcntl
            except ImportError as error:
                raise RuntimeError("single-instance locking requires Linux fcntl") from error
            self._flock_module = fcntl
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self.path.open("a+", encoding="ascii")
        try:
            self._flock_module.flock(
                self._handle.fileno(),
                self._flock_module.LOCK_EX | self._flock_module.LOCK_NB,
            )
        except BlockingIOError:
            self._handle.close()
            self._handle = None
            return False
        self._handle.seek(0)
        self._handle.truncate()
        self._handle.write(f"{os.getpid()}\n")
        self._handle.flush()
        return True

    def close(self) -> None:
        if self._handle is not None:
            self._handle.close()
            self._handle = None


def _default_lock_file() -> Path:
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR", "/tmp")
    return Path(runtime_dir) / "hball-edgetalk-usb.lock"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="explicit stable serial path; auto-detect by default")
    parser.add_argument("--lock-file", type=Path, default=_default_lock_file())
    parser.add_argument("--retry", type=float, default=DEFAULT_RETRY_S)
    parser.add_argument("--ping-rate", type=float, default=DEFAULT_PING_RATE_HZ)
    arguments = parser.parse_args()
    if arguments.retry <= 0.0 or arguments.ping_rate <= 0.0:
        parser.error("retry and ping-rate must be positive")

    lock = SingleInstanceLock(arguments.lock_file)
    try:
        if not lock.acquire():
            print("FAIL: another EdgeTalk USB daemon owns the serial link", file=sys.stderr)
            return 3

        def run_port_session(port: str) -> None:
            device = open_serial(port)
            try:
                run_safe_ping_session(device, ping_rate_hz=arguments.ping_rate)
            finally:
                device.close()

        supervise_link(
            lambda: find_serial_port(arguments.port),
            run_port_session,
            retry_s=arguments.retry,
        )
    except KeyboardInterrupt:
        return 0
    except RuntimeError as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 2
    finally:
        lock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
