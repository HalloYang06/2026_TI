import struct
import sys
from pathlib import Path

VISION_DIR = Path(__file__).resolve().parents[1]
if str(VISION_DIR) not in sys.path:
    sys.path.insert(0, str(VISION_DIR))
from edgetalk_log_protocol import ControlLogStream, crc32c, decode_frame


def make_frame(sequence=3):
    body = struct.pack(
        "<4sHHIIIIIIHHI9f", b"HBLG", 1, 80, sequence, 10, 11, 12, 13, 15,
        2, 0, 1, *([0.25] * 9),
    )
    return body + struct.pack("<I", crc32c(body))


def test_decode_and_resynchronize():
    frame = make_frame()
    record = decode_frame(frame)
    assert record.sequence == 3
    stream = ControlLogStream()
    assert stream.push(b"READY\n" + frame[:20]) == []
    records = stream.push(frame[20:])
    assert [item.sequence for item in records] == [3]
