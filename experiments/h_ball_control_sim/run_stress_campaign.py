from __future__ import annotations

from pathlib import Path

from hballsim.campaign import run_stress_campaign, write_campaign_outputs


OUTPUT = Path(__file__).resolve().parent / "output"


def main() -> None:
    campaign = run_stress_campaign(
        levels=(0.0, 1.0, 2.0, 3.0, 4.0, 5.0),
        trials_per_level=12,
        campaign_seed=20260729,
        duration=25.0,
    )
    write_campaign_outputs(campaign, OUTPUT)
    for summary in campaign.levels:
        print(
            f"level={summary.level:g} pass={summary.pass_count}/"
            f"{summary.trial_count} p95={summary.p95_peak_error_mm:.3f}mm "
            f"worst={summary.worst_peak_error_mm:.3f}mm"
        )


if __name__ == "__main__":
    main()
