from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = ROOT / "vision" / "raspberrypi" / "vision_measurement_protocol.py"
GOLDEN_FRAME = bytes.fromhex(
    "a55a010140003300403020100807060504030201c40900000080f642"
    "000036420000d8407b142ebd0000703f00000f431000140040017800"
    "8d200000d31952dc"
)


def load_protocol_module():
    spec = importlib.util.spec_from_file_location("vision_measurement_protocol", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_crc32c_uses_castagnoli_standard_check_value():
    protocol = load_protocol_module()

    assert protocol.crc32c(b"123456789") == 0xE3069283


def test_golden_frame_round_trips_exactly_across_python_and_c_contract():
    protocol = load_protocol_module()

    measurement = protocol.decode_measurement(GOLDEN_FRAME)

    assert measurement.sequence == 0x10203040
    assert measurement.capture_time_us == 0x0102030405060708
    assert measurement.processing_time_us == 2500
    assert measurement.center_x_px == 123.25
    assert measurement.center_y_px == 45.5
    assert measurement.radius_px == 6.75
    assert abs(measurement.ball_position_m - (-0.0425)) < 1e-8
    assert measurement.confidence == 0.9375
    assert measurement.contour_area_px2 == 143.0
    assert (measurement.roi_x, measurement.roi_y) == (16, 20)
    assert (measurement.roi_w, measurement.roi_h) == (320, 120)
    assert measurement.exposure_us == 8333
    assert protocol.encode_measurement(measurement) == GOLDEN_FRAME


def test_decoder_rejects_crc_damage_and_non_finite_measurements():
    protocol = load_protocol_module()
    damaged = bytearray(GOLDEN_FRAME)
    damaged[32] ^= 0x01

    try:
        protocol.decode_measurement(bytes(damaged))
    except protocol.FrameError as error:
        assert error.reason == "crc"
    else:
        raise AssertionError("CRC-damaged frame was accepted")

    measurement = protocol.decode_measurement(GOLDEN_FRAME)
    measurement.center_x_px = float("nan")
    try:
        protocol.encode_measurement(measurement)
    except ValueError:
        pass
    else:
        raise AssertionError("non-finite vision measurement was encoded")


def test_stream_decoder_resynchronizes_after_noise_crc_error_and_usb_splits():
    protocol = load_protocol_module()
    damaged = bytearray(GOLDEN_FRAME)
    damaged[24] ^= 0x80
    stream = protocol.VisionStreamDecoder()

    first = stream.feed(b"tty-noise" + bytes(damaged[:37]))
    second = stream.feed(bytes(damaged[37:]) + GOLDEN_FRAME[:11])
    third = stream.feed(GOLDEN_FRAME[11:] + GOLDEN_FRAME)

    assert first == []
    assert second == []
    assert [item.sequence for item in third] == [0x10203040, 0x10203040]
    assert stream.crc_failures == 1
    assert stream.frames_accepted == 2
