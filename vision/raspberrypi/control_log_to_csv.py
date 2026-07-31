#!/usr/bin/env python3
"""Convert CRC-valid EdgeTalk HBLG telemetry into Simulink-ready CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from edgetalk_log_protocol import (
    ControlLogStream,
    V1_VALUE_NAMES,
    V2_METADATA_NAMES,
    V2_VALUE_NAMES,
)


BASE_COLUMNS = (
    "time_s", "version", "frame_size", "sequence", "produced_time_ms", "sensor_sequence",
    "controller_steps", "vision_sequence", "sensor_valid_flags",
    "control_mode", "phase_or_guard_reason", "status_flags",
    "is_actual_control", "is_q3_actual", "control_active", "q3_passed",
)

VALUE_COLUMNS = V2_VALUE_NAMES + tuple(
    name for name in V1_VALUE_NAMES if name not in V2_VALUE_NAMES
)


def convert(
    input_path: Path,
    output_path: Path,
    actual_only: bool,
    q3_only: bool = False,
) -> dict[str, int]:
    decoder = ControlLogStream()
    records = []
    with input_path.open("rb") as source:
        while chunk := source.read(65536):
            records.extend(decoder.push(chunk))
    if actual_only:
        records = [record for record in records if record.is_actual_control]
    if q3_only:
        records = [record for record in records if record.is_q3_actual]
    if not records:
        raise ValueError("no matching CRC-valid HBLG records")

    origin_ms = records[0].produced_time_ms
    columns = BASE_COLUMNS + V2_METADATA_NAMES + VALUE_COLUMNS
    with output_path.open("w", newline="", encoding="utf-8") as destination:
        writer = csv.DictWriter(destination, fieldnames=columns)
        writer.writeheader()
        for record in records:
            named = record.named_values()
            row = {
                "time_s": (record.produced_time_ms - origin_ms) / 1000.0,
                "version": record.version,
                "frame_size": record.frame_size,
                "sequence": record.sequence,
                "produced_time_ms": record.produced_time_ms,
                "sensor_sequence": record.sensor_sequence,
                "controller_steps": record.controller_steps,
                "vision_sequence": record.vision_sequence,
                "sensor_valid_flags": record.sensor_valid_flags,
                "control_mode": record.control_mode,
                "phase_or_guard_reason": record.guard_reason,
                "status_flags": record.status_flags,
                "is_actual_control": int(record.is_actual_control),
                "is_q3_actual": int(record.is_q3_actual),
                "control_active": int(bool(record.status_flags & (1 << 17))),
                "q3_passed": int(bool(record.status_flags & (1 << 18))),
            }
            row.update(record.metadata)
            row.update(named)
            writer.writerow(row)
    return {
        "records": len(records),
        "crc_failures": decoder.crc_failures,
        "discarded_bytes": decoder.discarded_bytes,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="raw .hblg file")
    parser.add_argument("output", type=Path, help="output .csv file")
    parser.add_argument(
        "--all-sources", action="store_true",
        help="include M55 shadow frames; default keeps actual Q3-Q6 control",
    )
    parser.add_argument(
        "--q3-only", action="store_true",
        help="keep only deployed Q3 records",
    )
    arguments = parser.parse_args()
    stats = convert(
        arguments.input,
        arguments.output,
        not arguments.all_sources,
        arguments.q3_only,
    )
    print(" ".join(f"{key}={value}" for key, value in stats.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
