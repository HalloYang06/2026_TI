from __future__ import annotations

import csv
from dataclasses import replace
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from hballsim.simulation import SimulationConfig, run_scenario


OUTPUT = Path(__file__).resolve().parent / "output"


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    base_config = SimulationConfig()
    cases = {
        "baseline": base_config,
        "stress": replace(
            base_config,
            random_seed=20260730,
            camera_delay=0.050,
            camera_noise_std=0.0012,
            actuator_time_constant=0.040,
            actuator_rate_limit=math.radians(100.0),
            camera_dropout_windows=((11.80, 11.88), (19.90, 19.98)),
        ),
    }
    results = {name: run_scenario("lqg", config) for name, config in cases.items()}
    comparison_config = replace(base_config, initial_position=0.0)
    without_feedforward = run_scenario(
        "lqg_no_feedforward", comparison_config
    )
    with_feedforward = run_scenario("lqg", comparison_config)
    static_transfer = run_scenario(
        "lqg",
        replace(
            base_config,
            duration=5.0,
            random_seed=20260731,
            initial_position=0.0,
            vehicle_motion_enabled=False,
            target_schedule=((0.0, 0.050), (2.0, -0.050)),
        ),
    )
    arbitrary_position = run_scenario(
        "lqg",
        replace(
            base_config,
            random_seed=20260732,
            initial_position=0.050,
            target_position=0.050,
        ),
    )

    monte_carlo_rng = np.random.default_rng(20260729)
    monte_carlo_results = []
    for trial in range(20):
        trial_config = replace(
            cases["stress"],
            random_seed=20260800 + trial,
            initial_position=float(monte_carlo_rng.uniform(-0.004, 0.004)),
        )
        monte_carlo_results.append(run_scenario("lqg", trial_config))

    with (OUTPUT / "metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "scenario",
                "peak_error_mm",
                "rms_error_mm",
                "time_over_10mm_s",
                "max_beam_angle_deg",
                "final_error_mm",
            ]
        )
        for name, result in results.items():
            metrics = result.metrics
            writer.writerow(
                [
                    name,
                    metrics.peak_abs_error_m * 1000.0,
                    metrics.rms_error_m * 1000.0,
                    metrics.time_over_one_cm_s,
                    math.degrees(metrics.max_abs_beam_angle_rad),
                    metrics.final_error_m * 1000.0,
                ]
            )

    with (OUTPUT / "monte_carlo.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "trial",
                "initial_position_mm",
                "peak_error_mm",
                "rms_error_mm",
                "time_over_10mm_s",
            ]
        )
        for trial, result in enumerate(monte_carlo_results):
            writer.writerow(
                [
                    trial,
                    result.position[0] * 1000.0,
                    result.metrics.peak_abs_error_m * 1000.0,
                    result.metrics.rms_error_m * 1000.0,
                    result.metrics.time_over_one_cm_s,
                ]
            )

    baseline = results["baseline"]
    stress = results["stress"]
    figure, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

    axes[0].plot(baseline.time, baseline.position * 1000.0, label="actual")
    axes[0].plot(
        baseline.time,
        baseline.estimated_position * 1000.0,
        label="estimated",
        alpha=0.8,
    )
    axes[0].axhline(10.0, color="red", linestyle="--", linewidth=1)
    axes[0].axhline(-10.0, color="red", linestyle="--", linewidth=1)
    axes[0].set_ylabel("Baseline x / mm")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(stress.time, stress.position * 1000.0, label="actual")
    axes[1].plot(
        stress.time,
        stress.estimated_position * 1000.0,
        label="estimated",
        alpha=0.8,
    )
    axes[1].axhline(10.0, color="red", linestyle="--", linewidth=1)
    axes[1].axhline(-10.0, color="red", linestyle="--", linewidth=1)
    axes[1].set_ylabel("Stress x / mm")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(
        baseline.time,
        np.degrees(baseline.beam_angle),
        label="beam actual",
    )
    axes[2].plot(
        baseline.time,
        np.degrees(baseline.beam_command),
        label="beam command",
        alpha=0.75,
    )
    axes[2].axhline(4.0, color="red", linestyle="--", linewidth=1)
    axes[2].axhline(-4.0, color="red", linestyle="--", linewidth=1)
    axes[2].set_ylabel("Beam / deg")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(
        baseline.time,
        baseline.longitudinal_accel,
        label="longitudinal accel / m/s^2",
    )
    axes[3].plot(baseline.time, baseline.yaw_rate, label="yaw rate / rad/s")
    axes[3].set_ylabel("Vehicle disturbance")
    axes[3].set_xlabel("Time / s")
    axes[3].legend(loc="upper right")
    axes[3].grid(True, alpha=0.3)

    figure.suptitle("H-problem LQG feasibility study")
    figure.tight_layout()
    figure.savefig(OUTPUT / "lqr_feasibility.png", dpi=180)
    plt.close(figure)

    comparison_figure, comparison_axes = plt.subplots(
        2, 1, figsize=(11, 7), sharex=False
    )
    comparison_axes[0].plot(
        with_feedforward.time,
        with_feedforward.position * 1000.0,
        label="selected: nonlinear feedforward + LQG",
    )
    comparison_axes[0].plot(
        without_feedforward.time,
        without_feedforward.position * 1000.0,
        label="ablation: LQG without measured-disturbance feedforward",
        alpha=0.8,
    )
    comparison_axes[0].axhline(10.0, color="red", linestyle="--", linewidth=1)
    comparison_axes[0].axhline(-10.0, color="red", linestyle="--", linewidth=1)
    comparison_axes[0].set_ylabel("Ball error / mm")
    comparison_axes[0].set_xlabel("Time / s")
    comparison_axes[0].grid(True, alpha=0.3)
    comparison_axes[0].legend(loc="upper right")

    comparison_axes[1].plot(
        static_transfer.time,
        static_transfer.position * 1000.0,
        label="actual position",
    )
    comparison_axes[1].plot(
        static_transfer.time,
        static_transfer.target * 1000.0,
        label="commanded position",
        linestyle="--",
    )
    comparison_axes[1].fill_between(
        static_transfer.time,
        static_transfer.target * 1000.0 - 10.0,
        static_transfer.target * 1000.0 + 10.0,
        alpha=0.12,
        label="+-10 mm tolerance band",
    )
    comparison_axes[1].set_ylabel("Static transfer / mm")
    comparison_axes[1].set_xlabel("Time / s")
    comparison_axes[1].grid(True, alpha=0.3)
    comparison_axes[1].legend(loc="lower right")
    comparison_figure.suptitle("Algorithm ablation and requirement 3 check")
    comparison_figure.tight_layout()
    comparison_figure.savefig(OUTPUT / "algorithm_comparison.png", dpi=180)
    plt.close(comparison_figure)

    def settling_time(
        result, target: float, start: float, stop: float, tolerance: float
    ) -> float | None:
        interval = (result.time >= start) & (result.time < stop)
        interval_time = result.time[interval]
        interval_error = np.abs(result.position[interval] - target)
        bad = np.flatnonzero(interval_error > tolerance)
        if bad.size == 0:
            return start
        next_index = int(bad[-1]) + 1
        if next_index >= interval_time.size:
            return None
        return float(interval_time[next_index])

    plus_five_settling = settling_time(
        static_transfer, 0.050, 0.0, 2.0, 0.010
    )
    minus_five_settling = settling_time(
        static_transfer, -0.050, 2.0, 5.0, 0.010
    )
    with (OUTPUT / "requirements_metrics.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(["check", "value", "unit", "limit", "passed"])
        writer.writerow(
            [
                "plus_5cm_settling_time",
                plus_five_settling,
                "s",
                "<2.0",
                plus_five_settling is not None and plus_five_settling < 2.0,
            ]
        )
        writer.writerow(
            [
                "minus_5cm_settling_time",
                minus_five_settling,
                "s",
                "<5.0",
                minus_five_settling is not None and minus_five_settling < 5.0,
            ]
        )
        writer.writerow(
            [
                "moving_arbitrary_5cm_peak_error",
                arbitrary_position.metrics.peak_abs_error_m * 1000.0,
                "mm",
                "<=10",
                arbitrary_position.metrics.peak_abs_error_m <= 0.010,
            ]
        )

    with (OUTPUT / "summary.txt").open("w", encoding="utf-8") as stream:
        stream.write(f"LQR gain: {baseline.lqr_gain.tolist()}\n")
        stream.write(
            "Closed-loop poles: "
            f"{[complex(value) for value in baseline.closed_loop_eigenvalues]}\n"
        )
        for name, result in results.items():
            stream.write(f"{name}: {result.metrics}\n")
        stream.write(f"with_feedforward: {with_feedforward.metrics}\n")
        stream.write(f"without_feedforward: {without_feedforward.metrics}\n")
        stream.write(
            f"static_transfer_settling_s: +5cm={plus_five_settling}, "
            f"-5cm={minus_five_settling}\n"
        )
        monte_carlo_peaks = np.array(
            [result.metrics.peak_abs_error_m for result in monte_carlo_results]
        )
        monte_carlo_rms = np.array(
            [result.metrics.rms_error_m for result in monte_carlo_results]
        )
        stream.write(
            "Monte Carlo stress peak error (mm): "
            f"worst={np.max(monte_carlo_peaks) * 1000.0:.4f}, "
            f"p95={np.percentile(monte_carlo_peaks, 95) * 1000.0:.4f}\n"
        )
        stream.write(
            "Monte Carlo stress RMS error (mm): "
            f"worst={np.max(monte_carlo_rms) * 1000.0:.4f}, "
            f"p95={np.percentile(monte_carlo_rms, 95) * 1000.0:.4f}\n"
        )


if __name__ == "__main__":
    main()
