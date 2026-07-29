from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hballsim.campaign import run_stress_campaign, write_campaign_outputs


def test_small_stress_campaign_is_reproducible_and_summarized_by_level():
    first = run_stress_campaign(
        levels=(0.0, 1.0),
        trials_per_level=2,
        campaign_seed=20260729,
        duration=1.0,
    )
    second = run_stress_campaign(
        levels=(0.0, 1.0),
        trials_per_level=2,
        campaign_seed=20260729,
        duration=1.0,
    )

    assert first.trials == second.trials
    assert first.levels == second.levels
    assert len(first.trials) == 4
    assert len(first.levels) == 2
    assert all(0.0 <= level.pass_rate <= 1.0 for level in first.levels)


def test_campaign_outputs_include_machine_readable_results_and_plot(tmp_path):
    campaign = run_stress_campaign(
        levels=(0.0,),
        trials_per_level=2,
        campaign_seed=20260729,
        duration=1.0,
    )

    write_campaign_outputs(campaign, tmp_path)

    assert (tmp_path / "stress_trials.csv").is_file()
    assert (tmp_path / "stress_levels.csv").is_file()
    assert (tmp_path / "stress_summary.txt").is_file()
    assert (tmp_path / "stress_envelope.png").stat().st_size > 10_000
