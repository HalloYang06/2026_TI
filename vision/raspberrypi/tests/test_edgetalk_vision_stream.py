from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
VISION_DIR = ROOT / "vision" / "raspberrypi"
MODULE_PATH = VISION_DIR / "edgetalk_vision_stream.py"


def load_stream_module():
    sys.path.insert(0, str(VISION_DIR))
    spec = importlib.util.spec_from_file_location("edgetalk_vision_stream", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_200_hz_synthetic_stream_writes_monotonic_valid_64_byte_frames():
    stream = load_stream_module()

    class FakeClock:
        def __init__(self):
            self.now = 100.0

        def monotonic(self):
            return self.now

        def sleep(self, duration):
            self.now += duration

    class FakeSerial:
        def __init__(self, clock):
            self.clock = clock
            self.frames = []
            self.flushed = 0

        def write(self, data):
            self.frames.append(data)
            self.clock.now += 0.0002
            return len(data)

        def flush(self):
            self.flushed += 1

    clock = FakeClock()
    serial_port = FakeSerial(clock)
    result = stream.run_stream(
        serial_port, duration_s=0.05, rate_hz=200.0,
        monotonic=clock.monotonic, sleep=clock.sleep,
    )

    decoded = [stream.decode_measurement(frame) for frame in serial_port.frames]
    assert len(decoded) == 10
    assert [item.sequence for item in decoded] == list(range(10))
    assert all(len(frame) == 64 for frame in serial_port.frames)
    assert result["tx_frames"] == 10
    assert result["tx_bytes"] == 640
    assert result["deadline_misses"] == 0
    assert result["achieved_rate_hz"] == 200.0
    assert result["payload_rate_bytes_s"] == 12800.0
    assert serial_port.flushed == 1


def test_stream_rate_verdict_requires_requested_minimum():
    stream = load_stream_module()

    assert stream.rate_passes(
        {"achieved_rate_hz": 239.5, "deadline_misses": 0}, 230.0
    )
    assert not stream.rate_passes(
        {"achieved_rate_hz": 119.9, "deadline_misses": 0}, 120.0
    )
    assert not stream.rate_passes(
        {"achieved_rate_hz": 240.0, "deadline_misses": 1}, 120.0
    )


def test_stream_rejects_non_positive_duration_or_rate():
    stream = load_stream_module()

    for duration, rate in ((0.0, 120.0), (1.0, 0.0)):
        try:
            stream.run_stream(object(), duration, rate)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid stream timing was accepted")


def test_real_serial_stream_uses_bounded_20ms_write_timeout():
    source = MODULE_PATH.read_text(encoding="utf-8")

    assert "VISION_WRITE_TIMEOUT_S = 0.020" in source
    assert "write_timeout=VISION_WRITE_TIMEOUT_S" in source
