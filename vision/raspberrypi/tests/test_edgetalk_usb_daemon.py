from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
VISION_DIR = ROOT / "vision" / "raspberrypi"
MODULE_PATH = VISION_DIR / "edgetalk_usb_daemon.py"
SERVICE_PATH = VISION_DIR / "systemd" / "hball-edgetalk-usb.service"


def load_daemon_module():
    sys.path.insert(0, str(VISION_DIR))
    spec = importlib.util.spec_from_file_location("edgetalk_usb_daemon", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_stable_by_id_port_is_preferred_over_ttyacm(tmp_path):
    daemon = load_daemon_module()
    by_id = tmp_path / "serial" / "by-id"
    dev = tmp_path / "dev"
    by_id.mkdir(parents=True)
    dev.mkdir()
    stable = by_id / "usb-Infineon_HBall_CDC-if00"
    stable.touch()
    (dev / "ttyACM0").touch()

    selected = daemon.find_serial_port(None, by_id_dir=by_id, dev_dir=dev)

    assert selected == str(stable)


def test_supervisor_waits_for_device_and_reconnects_after_disconnect():
    daemon = load_daemon_module()
    ports = iter([None, "/dev/serial/by-id/hball", "/dev/serial/by-id/hball"])
    session_outcomes = iter([OSError("USB disconnected"), None])
    sleeps: list[float] = []

    def resolve_port():
        port = next(ports)
        if port is None:
            raise RuntimeError("device absent")
        return port

    def run_session(_port):
        outcome = next(session_outcomes)
        if outcome is not None:
            raise outcome

    result = daemon.supervise_link(
        resolve_port,
        run_session,
        retry_s=0.25,
        sleep=sleeps.append,
        max_cycles=3,
    )

    assert result == {
        "cycles": 3,
        "device_waits": 1,
        "connections": 2,
        "disconnects": 1,
        "sessions_completed": 1,
    }
    assert sleeps == [0.25, 0.25, 0.25]


def test_serial_is_opened_exclusively_and_second_instance_is_rejected(tmp_path):
    daemon = load_daemon_module()
    serial_arguments = {}

    def serial_factory(**arguments):
        serial_arguments.update(arguments)
        return object()

    daemon.open_serial("/dev/serial/by-id/hball", serial_factory=serial_factory)
    assert serial_arguments["exclusive"] is True
    assert serial_arguments["write_timeout"] <= 1.0

    class FakeFcntl:
        LOCK_EX = 1
        LOCK_NB = 2
        locked = False

        @classmethod
        def flock(cls, _file_descriptor, _flags):
            if cls.locked:
                raise BlockingIOError("already locked")
            cls.locked = True

    first = daemon.SingleInstanceLock(tmp_path / "daemon.lock", flock_module=FakeFcntl)
    second = daemon.SingleInstanceLock(tmp_path / "daemon.lock", flock_module=FakeFcntl)
    assert first.acquire()
    assert not second.acquire()
    first.close()


def test_startup_session_emits_only_separator_and_safe_ping_lines(capsys):
    daemon = load_daemon_module()

    class FakeClock:
        def __init__(self):
            self.now = 10.0

        def monotonic(self):
            self.now += 0.001
            return self.now

        def sleep(self, duration):
            self.now += duration

    class FakeSerial:
        def __init__(self):
            self.writes: list[bytes] = []
            self.lines = [b"HBALL_USB_READY 0.1.0 7 1200\n"]

        def reset_input_buffer(self):
            self.lines = [b"HBALL_USB_READY 0.1.0 7 1200\n"]

        def write(self, data):
            self.writes.append(data)
            if data.startswith(b"PING "):
                self.lines.append(data.replace(b"PING ", b"PONG ", 1))
            return len(data)

        def flush(self):
            pass

        def readline(self):
            return self.lines.pop(0) if self.lines else b""

    clock = FakeClock()
    serial_port = FakeSerial()
    result = daemon.run_safe_ping_session(
        serial_port,
        ready_timeout_s=0.1,
        ping_timeout_s=0.1,
        ping_rate_hz=10.0,
        max_pings=2,
        monotonic=clock.monotonic,
        sleep=clock.sleep,
    )

    assert result["ready_sequence"] == 7
    assert result["pings"] == 2
    assert "READY: sequence=7 uptime_ms=1200" in capsys.readouterr().out
    assert serial_port.writes[0] == b"\n"
    assert all(message.startswith(b"PING ") for message in serial_port.writes[1:])
    assert all(len(message) != 64 for message in serial_port.writes)
    source = MODULE_PATH.read_text(encoding="utf-8")
    assert "build_synthetic_measurement" not in source
    assert "FLAG_POSITION_VALID" not in source


def test_systemd_restarts_always_without_identity_or_device_hardcoding():
    service = SERVICE_PATH.read_text(encoding="utf-8")

    assert "Restart=always" in service
    assert "EnvironmentFile=%h/.config/hball/edgetalk-usb.env" in service
    assert "${HBALL_USB_DAEMON}" in service
    assert "edgetalk_vision_stream.py" not in service
    assert "User=" not in service
    for forbidden in ("halloyang", "192.168.", "ttyACM", "password", "serial/by-id/usb-"):
        assert forbidden not in service
