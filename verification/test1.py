"""
Collect data first: `python collect.py A` (creates data/run_A_*.json).
Run from this directory: `pytest test1.py -v`
"""

from __future__ import annotations

import pytest

from evaluators import check_paths

# Run A in collect.configs_for("A") always schedules 100 trials (trial_number 1..100).
_TRIAL_NUMBERS = list(range(1, 101))


@pytest.mark.verification_suite("1A")
@pytest.mark.parametrize(
    "trial_number",
    _TRIAL_NUMBERS,
    ids=[f"trial_{n}" for n in _TRIAL_NUMBERS],
)
def test1A(run_a, trial_number: int):
    """One node per trial; failed nodes identify the case id in brackets."""
    trial = next((t for t in run_a if t.trial_number == trial_number), None)
    assert trial is not None, (
        f"no trial with trial_number={trial_number} in run A data "
        f"(expected {len(_TRIAL_NUMBERS)} files under data/)"
    )
    ok, reason = check_paths(trial)
    assert ok, f"trial {trial_number} (seed={trial.seed}): {reason}"
