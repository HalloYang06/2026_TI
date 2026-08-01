#!/usr/bin/env python3
"""Apply one EdgeTalk ball-control parameter through the running Pi bridge."""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path


PARAMETERS = {
    "kp", "kv", "ki", "run_deg", "run_recovery_deg", "settle_deg",
    "settle_capture_deg", "settle_recovery_deg", "q3_kp", "q3_ki",
    "q3_kd", "q3_rate_cms", "q3_level_mrad",
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", choices=sorted(PARAMETERS))
    parser.add_argument("value", type=float)
    parser.add_argument("--command-file", default="/tmp/hball-edgetalk-tune.cmd")
    parser.add_argument("--ack-file", default="/tmp/hball-edgetalk-tune.ack")
    parser.add_argument("--timeout", type=float, default=2.0)
    arguments = parser.parse_args()

    sequence = int(time.time() * 1000.0) & 0xFFFFFFFF
    command = f"HBALL_TUNE {sequence} {arguments.name} {arguments.value:.7g}\n"
    command_path = Path(arguments.command_file)
    temporary = command_path.with_suffix(f".tmp.{os.getpid()}")
    temporary.write_text(command, encoding="ascii")
    temporary.replace(command_path)

    deadline = time.monotonic() + arguments.timeout
    expected = f"HBALL_TUNE_ACK {sequence} "
    while time.monotonic() < deadline:
        try:
            ack = Path(arguments.ack_file).read_text(encoding="ascii")
        except FileNotFoundError:
            ack = ""
        if ack.startswith(expected):
            print(ack.strip())
            return 0 if ack.rstrip().endswith(" OK") else 2
        time.sleep(0.02)
    print("TIMEOUT: EdgeTalk did not acknowledge tuning command")
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
