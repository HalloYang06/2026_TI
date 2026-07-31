import csv
import struct
import sys
from pathlib import Path

VISION_DIR = Path(__file__).resolve().parents[1]
if str(VISION_DIR) not in sys.path:
    sys.path.insert(0, str(VISION_DIR))
from edgetalk_log_protocol import (
    ControlLogStream,
    STATUS_ACTUAL_CONTROL,
    V2_VALUE_NAMES,
    crc32c,
    decode_frame,
)
from control_log_to_csv import convert


def make_frame(sequence=3):
    body = struct.pack(
        "<4sHHIIIIIIHHI9f", b"HBLG", 1, 80, sequence, 10, 11, 12, 13, 15,
        2, 0, 1, *([0.25] * 9),
    )
    return body + struct.pack("<I", crc32c(body))


def make_v2_frame(sequence=4):
    prefix = struct.pack(
        "<4sHH6IHHIQIIHH5I4HIIHH4B6I",
        b"HBLG", 2, 288,
        sequence, 1000, 20, 30, STATUS_ACTUAL_CONTROL, 0x7f,
        2, 0, 40, 123456789, 900, 1200, 12, 0x0f,
        800, 901, 902, 903, 904,
        50, 12, 12, 12, 905, 906,
        0x0e, 0x03, 0, 2, 5, 0,
        7, 8, 9, 10, 11, 12,
    )
    values = struct.pack("<39f", *[float(index) for index in range(39)])
    body = prefix + values
    return body + struct.pack("<I", crc32c(body))


def test_decode_and_resynchronize():
    frame = make_frame()
    record = decode_frame(frame)
    assert record.sequence == 3
    stream = ControlLogStream()
    assert stream.push(b"READY\n" + frame[:20]) == []
    records = stream.push(frame[20:])
    assert [item.sequence for item in records] == [3]


def test_decode_v2_preserves_source_and_receive_timestamps():
    frame = make_v2_frame()
    record = decode_frame(frame)

    assert record.version == 2
    assert record.frame_size == 288
    assert record.is_actual_control
    assert record.metadata["vision_capture_time_us"] == 123456789
    assert record.metadata["vision_receive_time_ms"] == 900
    assert record.metadata["imu_source_time_ms"] == 800
    assert record.metadata["accel_receive_time_ms"] == 901
    assert record.metadata["motor_receive_time_ms"] == 906
    assert record.value_names == V2_VALUE_NAMES
    assert record.named_values()["accel_z_mps2"] == 17.0


def test_stream_resynchronizes_across_mixed_v1_and_v2_frames():
    v1 = make_frame(7)
    v2 = make_v2_frame(8)
    stream = ControlLogStream()

    records = stream.push(b"noise" + v1 + v2)
    assert [(record.version, record.sequence) for record in records] == [
        (1, 7), (2, 8),
    ]


def test_v2_csv_exports_vision_imu_motor_and_controller_fields(tmp_path):
    raw_path = tmp_path / "run.hblg"
    csv_path = tmp_path / "run.csv"
    raw_path.write_bytes(make_v2_frame())

    stats = convert(raw_path, csv_path, actual_only=True)
    with csv_path.open(newline="", encoding="utf-8") as source:
        row = next(csv.DictReader(source))

    assert stats["records"] == 1
    assert row["vision_capture_time_us"] == "123456789"
    assert row["imu_source_time_ms"] == "800"
    assert row["motor_receive_time_ms"] == "906"
    assert row["accel_z_mps2"] == "17.0"
    assert row["controller_command_rad"] == "33.0"
