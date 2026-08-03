#!/usr/bin/env python3
"""Prepare one Raspberry Pi Q6 control.csv for MATLAB/Simulink replay."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


IMU_VALID = 1 << 1
CONTROL_ACTIVE = 1 << 17


def number(row: dict[str, str], name: str) -> float:
    try:
        return float(row[name])
    except (KeyError, TypeError, ValueError):
        return math.nan


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    source = args.run_dir / "control.csv"
    output = args.run_dir / "q6_simulink.csv"
    metrics_path = args.run_dir / "q6_metrics.json"

    with source.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    rows = [
        row for row in rows
        if int(number(row, "status_flags")) & CONTROL_ACTIVE
    ]
    if len(rows) < 2:
        raise SystemExit("need at least two active control records")

    for row in rows:
        row["target_position_m"] = row["estimated_disturbance_mps2"]
    fields = list(rows[0])
    if "target_position_m" not in fields:
        fields.append("target_position_m")
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    times = [number(row, "time_s") for row in rows]
    targets = [number(row, "target_position_m") for row in rows]
    positions = [number(row, "ball_position_m") for row in rows]
    pipes = [number(row, "pipe_target_rad") for row in rows]
    imu_valid = [
        bool(int(number(row, "sensor_valid_flags")) & IMU_VALID)
        for row in rows
    ]
    steady_start = times[-1] - min(3.0, 0.25 * (times[-1] - times[0]))
    steady_indexes = [i for i, value in enumerate(times) if value >= steady_start]
    steady_errors = [targets[i] - positions[i] for i in steady_indexes]
    dropout_events = sum(
        previous and not current
        for previous, current in zip([True] + imu_valid[:-1], imu_valid)
    )
    longest_dropout = 0
    current_dropout = 0
    for valid in imu_valid:
        current_dropout = 0 if valid else current_dropout + 1
        longest_dropout = max(longest_dropout, current_dropout)
    sample_period = (times[-1] - times[0]) / (len(times) - 1)
    metrics = {
        "records": len(rows),
        "duration_s": times[-1] - times[0],
        "sample_rate_hz": 1.0 / sample_period,
        "target_cm": 100.0 * sum(targets[-len(steady_indexes):]) / len(steady_indexes),
        "steady_position_cm": 100.0 * sum(positions[i] for i in steady_indexes) / len(steady_indexes),
        "steady_error_cm": 100.0 * sum(steady_errors) / len(steady_errors),
        "max_abs_error_cm": 100.0 * max(abs(t - p) for t, p in zip(targets, positions)),
        "max_pipe_deg": math.degrees(max(abs(value) for value in pipes)),
        "imu_valid_fraction": sum(imu_valid) / len(imu_valid),
        "imu_dropout_events": dropout_events,
        "imu_longest_dropout_ms": 1000.0 * longest_dropout * sample_period,
    }
    metrics_path.write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(metrics, ensure_ascii=False))
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
