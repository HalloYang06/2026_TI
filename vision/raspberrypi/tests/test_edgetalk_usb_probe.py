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
