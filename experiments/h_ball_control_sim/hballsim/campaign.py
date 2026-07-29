from __future__ import annotations

import csv
from dataclasses import asdict, dataclass
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .simulation import SimulationConfig, SimulationResult, run_scenario
from .stress import generate_stress_case


ERROR_LIMIT_M = 0.010


@dataclass(frozen=True)
class StressTrial:
    level: float
    trial: int
    seed: int
    passed: bool
    peak_error_mm: float
    rms_error_mm: float
    first_failure_s: float | None
    failure_reason: str
    max_beam_angle_deg: float
    camera_delay_ms: float
    camera_noise_mm: float
    camera_outlier_probability: float
    longest_camera_dropout_ms: float
    imu_delay_ms: float
    imu_accel_bias: float
    state_packet_dropout_probability: float
    actuator_time_constant_ms: float
    actuator_command_delay_ms: float
    actuator_gain: float
    actuator_deadband_deg: float
    largest_impact_impulse_mps: float
    viscous_damping: float
    coulomb_accel: float
    static_friction_accel: float


@dataclass(frozen=True)
class StressLevelSummary:
    level: float
    trial_count: int
    pass_count: int
    pass_rate: float
    p95_peak_error_mm: float
    worst_peak_error_mm: float
    worst_rms_error_mm: float
    first_failed_trial: int | None


@dataclass(frozen=True)
class StressCampaign:
    trials: tuple[StressTrial, ...]
    levels: tuple[StressLevelSummary, ...]
    campaign_seed: int
    duration: float
    error_limit_mm: float = ERROR_LIMIT_M * 1000.0


def _first_failure_time(result: SimulationResult) -> float | None:
    error = np.abs(result.position - result.target)
    failed_indices = np.flatnonzero(error > ERROR_LIMIT_M)
    if failed_indices.size == 0:
        return None
    return float(result.time[int(failed_indices[0])])


def _classify_failure(
    config: SimulationConfig,
    result: SimulationResult,
    first_failure_s: float | None,
) -> str:
    """Return a deterministic first-order cause label, not a causal proof."""

    if first_failure_s is None:
        return "none"
    for start, duration, _ in config.impact_events:
        if start - 0.05 <= first_failure_s <= start + duration + 0.35:
            return "track_impact"

    time_window = np.abs(result.time - first_failure_s) <= 0.20
    if np.any(
        np.abs(result.beam_angle[time_window]) >= 0.95 * config.beam_limit
    ):
        return "actuator_saturation_or_lag"

    for start, stop in config.camera_dropout_windows:
        if start - 0.50 <= first_failure_s <= stop + config.camera_delay + 0.25:
            return "vision_delay_or_dropout"
    if config.camera_delay >= 0.080 or config.camera_outlier_probability >= 0.02:
        return "vision_delay_or_outlier"
    if (
        config.state_packet_dropout_probability >= 0.08
        or abs(config.imu_accel_bias) >= 0.08
    ):
        return "imu_or_state_link"
    return "friction_or_model_mismatch"


def _trial_from_result(
    *,
    level: float,
    trial: int,
    config: SimulationConfig,
    result: SimulationResult,
) -> StressTrial:
    first_failure_s = _first_failure_time(result)
    longest_dropout = max(
        (stop - start for start, stop in config.camera_dropout_windows),
        default=0.0,
    )
    largest_impact_impulse = max(
        (abs(duration * amplitude) for _, duration, amplitude in config.impact_events),
        default=0.0,
    )
    return StressTrial(
        level=level,
        trial=trial,
        seed=config.random_seed,
        passed=first_failure_s is None,
        peak_error_mm=result.metrics.peak_abs_error_m * 1000.0,
        rms_error_mm=result.metrics.rms_error_m * 1000.0,
        first_failure_s=first_failure_s,
        failure_reason=_classify_failure(config, result, first_failure_s),
        max_beam_angle_deg=math.degrees(result.metrics.max_abs_beam_angle_rad),
        camera_delay_ms=config.camera_delay * 1000.0,
        camera_noise_mm=config.camera_noise_std * 1000.0,
        camera_outlier_probability=config.camera_outlier_probability,
        longest_camera_dropout_ms=longest_dropout * 1000.0,
        imu_delay_ms=config.imu_delay * 1000.0,
        imu_accel_bias=config.imu_accel_bias,
        state_packet_dropout_probability=config.state_packet_dropout_probability,
        actuator_time_constant_ms=config.actuator_time_constant * 1000.0,
        actuator_command_delay_ms=config.actuator_command_delay * 1000.0,
        actuator_gain=config.actuator_gain,
        actuator_deadband_deg=math.degrees(config.actuator_deadband),
        largest_impact_impulse_mps=largest_impact_impulse,
        viscous_damping=config.actual_viscous_damping,
        coulomb_accel=config.actual_coulomb_accel,
        static_friction_accel=config.actual_static_friction_accel,
    )


def _summarize_level(level: float, trials: list[StressTrial]) -> StressLevelSummary:
    peaks = np.array([trial.peak_error_mm for trial in trials], dtype=float)
    rms_values = np.array([trial.rms_error_mm for trial in trials], dtype=float)
    failed_trials = [trial.trial for trial in trials if not trial.passed]
    pass_count = sum(trial.passed for trial in trials)
    return StressLevelSummary(
        level=level,
        trial_count=len(trials),
        pass_count=pass_count,
        pass_rate=pass_count / len(trials),
        p95_peak_error_mm=float(np.percentile(peaks, 95)),
        worst_peak_error_mm=float(np.max(peaks)),
        worst_rms_error_mm=float(np.max(rms_values)),
        first_failed_trial=min(failed_trials) if failed_trials else None,
    )


def run_stress_campaign(
    *,
    levels: tuple[float, ...],
    trials_per_level: int,
    campaign_seed: int,
    duration: float = 25.0,
) -> StressCampaign:
    if not levels:
        raise ValueError("levels must not be empty")
    if trials_per_level <= 0:
        raise ValueError("trials_per_level must be positive")

    all_trials: list[StressTrial] = []
    level_summaries: list[StressLevelSummary] = []
    for level in levels:
        level_trials = []
        for trial in range(trials_per_level):
            config = generate_stress_case(
                level=level,
                trial=trial,
                campaign_seed=campaign_seed,
                duration=duration,
            )
            result = run_scenario("lqg", config)
            trial_result = _trial_from_result(
                level=level,
                trial=trial,
                config=config,
                result=result,
            )
            level_trials.append(trial_result)
            all_trials.append(trial_result)
        level_summaries.append(_summarize_level(level, level_trials))

    return StressCampaign(
        trials=tuple(all_trials),
        levels=tuple(level_summaries),
        campaign_seed=campaign_seed,
        duration=duration,
    )


def _write_dataclass_csv(path: Path, rows: tuple[object, ...]) -> None:
    dictionaries = [asdict(row) for row in rows]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(dictionaries[0]))
        writer.writeheader()
        writer.writerows(dictionaries)


def _rank_sensitivities(campaign: StressCampaign) -> list[tuple[str, float]]:
    feature_names = [
        "camera_delay_ms",
        "camera_noise_mm",
        "longest_camera_dropout_ms",
        "imu_delay_ms",
        "imu_accel_bias",
        "state_packet_dropout_probability",
        "actuator_time_constant_ms",
        "actuator_command_delay_ms",
        "actuator_gain",
        "actuator_deadband_deg",
        "largest_impact_impulse_mps",
        "viscous_damping",
        "coulomb_accel",
        "static_friction_accel",
    ]
    target = np.array([trial.peak_error_mm for trial in campaign.trials])
    ranked = []
    for name in feature_names:
        values = np.array([getattr(trial, name) for trial in campaign.trials])
        if np.std(values) <= 1e-12 or np.std(target) <= 1e-12:
            correlation = 0.0
        else:
            correlation = float(np.corrcoef(values, target)[0, 1])
        ranked.append((name, correlation))
    return sorted(ranked, key=lambda item: abs(item[1]), reverse=True)


def write_campaign_outputs(campaign: StressCampaign, output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    _write_dataclass_csv(output / "stress_trials.csv", campaign.trials)
    _write_dataclass_csv(output / "stress_levels.csv", campaign.levels)

    levels = np.array([summary.level for summary in campaign.levels])
    pass_rates = np.array([summary.pass_rate for summary in campaign.levels])
    p95_peaks = np.array(
        [summary.p95_peak_error_mm for summary in campaign.levels]
    )
    worst_peaks = np.array(
        [summary.worst_peak_error_mm for summary in campaign.levels]
    )

    figure, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(levels, pass_rates * 100.0, marker="o", label="pass rate")
    axes[0].set_ylabel("Pass rate / %")
    axes[0].set_ylim(-5.0, 105.0)
    axes[0].grid(True, alpha=0.3)
    for level, rate in zip(levels, pass_rates, strict=True):
        axes[0].annotate(
            f"{rate * 100.0:.0f}%",
            (level, rate * 100.0),
            xytext=(0, 6),
            textcoords="offset points",
            ha="center",
        )

    axes[1].plot(levels, p95_peaks, marker="o", label="P95 peak error")
    axes[1].plot(levels, worst_peaks, marker="s", label="worst peak error")
    axes[1].axhline(
        campaign.error_limit_mm,
        color="red",
        linestyle="--",
        linewidth=1.2,
        label="10 mm requirement",
    )
    axes[1].set_xlabel("Stress level")
    axes[1].set_ylabel("Peak position error / mm")
    axes[1].set_yscale("log")
    axes[1].set_ylim(2.5, 150.0)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper left")
    figure.suptitle("H-problem LQG graded stress envelope")
    figure.tight_layout()
    figure.savefig(output / "stress_envelope.png", dpi=180)
    plt.close(figure)

    failed = [trial for trial in campaign.trials if not trial.passed]
    first_failing_level = next(
        (summary.level for summary in campaign.levels if summary.pass_rate < 1.0),
        None,
    )
    if first_failing_level is not None:
        boundary_trial = max(
            (
                trial
                for trial in campaign.trials
                if trial.level == first_failing_level and not trial.passed
            ),
            key=lambda trial: trial.peak_error_mm,
        )
        boundary_config = generate_stress_case(
            level=boundary_trial.level,
            trial=boundary_trial.trial,
            campaign_seed=campaign.campaign_seed,
            duration=campaign.duration,
        )
        boundary_result = run_scenario("lqg", boundary_config)
        failure_time = boundary_trial.first_failure_s or 0.0
        window_start = max(0.0, failure_time - 2.0)
        window_stop = min(campaign.duration, failure_time + 2.0)
        boundary_figure, boundary_axes = plt.subplots(
            2, 1, figsize=(11, 7), sharex=True
        )
        boundary_axes[0].plot(
            boundary_result.time,
            boundary_result.position * 1000.0,
            label="actual position",
        )
        boundary_axes[0].plot(
            boundary_result.time,
            boundary_result.estimated_position * 1000.0,
            label="estimated position",
            alpha=0.8,
        )
        boundary_axes[0].axhline(10.0, color="red", linestyle="--", linewidth=1)
        boundary_axes[0].axhline(-10.0, color="red", linestyle="--", linewidth=1)
        boundary_axes[0].set_ylabel("Ball error / mm")
        boundary_axes[0].legend(loc="upper left")
        boundary_axes[0].grid(True, alpha=0.3)

        boundary_axes[1].plot(
            boundary_result.time,
            np.degrees(boundary_result.beam_angle),
            label="actual beam",
        )
        boundary_axes[1].plot(
            boundary_result.time,
            np.degrees(boundary_result.beam_command),
            label="beam command",
            alpha=0.8,
        )
        boundary_axes[1].axhline(4.0, color="red", linestyle="--", linewidth=1)
        boundary_axes[1].axhline(-4.0, color="red", linestyle="--", linewidth=1)
        boundary_axes[1].set_ylabel("Beam angle / deg")
        boundary_axes[1].set_xlabel("Time / s")
        boundary_axes[1].legend(loc="upper left")
        boundary_axes[1].grid(True, alpha=0.3)

        for axis in boundary_axes:
            axis.set_xlim(window_start, window_stop)
            for start, stop in boundary_config.camera_dropout_windows:
                if stop >= window_start and start <= window_stop:
                    axis.axvspan(
                        start,
                        stop,
                        color="gray",
                        alpha=0.15,
                        label=None,
                    )
            for start, _, _ in boundary_config.impact_events:
                if window_start <= start <= window_stop:
                    axis.axvline(start, color="orange", alpha=0.65, linewidth=1)
            axis.axvline(failure_time, color="red", alpha=0.8, linewidth=1.2)
        boundary_figure.suptitle(
            f"First boundary failure: level {boundary_trial.level:g}, "
            f"trial {boundary_trial.trial}, seed {boundary_trial.seed}"
        )
        boundary_figure.tight_layout()
        boundary_figure.savefig(output / "boundary_failure.png", dpi=180)
        plt.close(boundary_figure)

    with (output / "stress_summary.txt").open("w", encoding="utf-8") as stream:
        stream.write(
            f"seed={campaign.campaign_seed}, duration={campaign.duration:.3f}s, "
            f"limit={campaign.error_limit_mm:.3f}mm\n"
        )
        stream.write(f"first_failing_level={first_failing_level}\n")
        for summary in campaign.levels:
            stream.write(
                f"level={summary.level:g}: pass={summary.pass_count}/"
                f"{summary.trial_count}, p95_peak={summary.p95_peak_error_mm:.3f}mm, "
                f"worst_peak={summary.worst_peak_error_mm:.3f}mm, "
                f"worst_rms={summary.worst_rms_error_mm:.3f}mm\n"
            )
        if failed:
            worst = max(failed, key=lambda trial: trial.peak_error_mm)
            stream.write(
                f"worst_failure=level {worst.level:g}, trial {worst.trial}, "
                f"seed {worst.seed}, peak {worst.peak_error_mm:.3f}mm, "
                f"time {worst.first_failure_s:.3f}s, reason {worst.failure_reason}\n"
            )
        stream.write("sensitivity_correlations_with_peak_error:\n")
        for name, correlation in _rank_sensitivities(campaign)[:8]:
            stream.write(f"  {name}: {correlation:+.3f}\n")
