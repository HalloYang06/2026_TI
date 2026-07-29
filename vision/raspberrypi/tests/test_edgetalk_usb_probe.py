from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = ROOT / "vision" / "raspberrypi" / "edgetalk_usb_probe.py"


def load_probe_module():
    spec = importlib.util.spec_from_file_location("edgetalk_usb_probe", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_build_ping_and_parse_matching_pong():
    probe = load_probe_module()

    request = probe.build_ping(17, "host_ms=123 payload ok")
    reply = probe.parse_pong(b"PONG 17 host_ms=123 payload ok\r\n")

    assert request == b"PING 17 host_ms=123 payload ok\n"
    assert reply == (17, "host_ms=123 payload ok")


def test_parse_ready_rejects_wrong_version_or_malformed_lines():
    probe = load_probe_module()

    assert probe.parse_ready(b"HBALL_USB_READY 0.1.0 4 8123\n") == (4, 8123)
    assert probe.parse_ready(b"HBALL_USB_READY 9.9.9 4 8123\n") is None
    assert probe.parse_ready(b"MOTOR_ENABLE 1\n") is None
    assert probe.parse_pong(b"PONG 4294967296 overflow\n") is None


def test_payload_contract_blocks_newlines_and_oversize_data():
    probe = load_probe_module()

    for payload in ("bad\nline", "x" * 49):
        try:
            probe.build_ping(1, payload)
        except ValueError:
            pass
        else:
            raise AssertionError("unsafe probe payload was accepted")


def test_link_sync_sends_separator_before_waiting_for_fresh_ready():
    probe = load_probe_module()

    class FakeSerial:
        def __init__(self):
            self.writes = []
            self.flushed = 0
            self.lines = [b"HBALL_USB_READY 0.1.0 7 1234\n"]

        def reset_input_buffer(self):
            self.lines = [b"HBALL_USB_READY 0.1.0 7 1234\n"]

        def write(self, data):
            self.writes.append(data)
            return len(data)

        def flush(self):
            self.flushed += 1

        def readline(self):
            return self.lines.pop(0) if self.lines else b""

    serial_port = FakeSerial()
    ready = probe._synchronize_link(serial_port, 0.1)

    assert serial_port.writes == [b"\n"]
    assert serial_port.flushed == 1
    assert ready == (7, 1234)
