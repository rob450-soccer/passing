"""
Collect data first: `python collect.py C` (creates data/run_C_*.json).
Run from this directory: `pytest test6B.py -v`
"""

from __future__ import annotations

import pytest

from evaluators import check_kick_torque

_TRIAL_NUMBERS = list(range(1, 101))


@pytest.mark.verification_suite("6B")
@pytest.mark.parametrize(
    "trial_number",
    _TRIAL_NUMBERS,
    ids=[f"trial_{n}" for n in _TRIAL_NUMBERS],
)
def test6B(run_c, trial_number: int):
    """Test 5B: passing torque."""
    trial = next((t for t in run_c if t.trial_number == trial_number), None)
    assert trial is not None, (
        f"no trial with trial_number={trial_number} in run C data"
        f"(expected {len(_TRIAL_NUMBERS)} files under data/)"
    )

    ok, reason = check_kick_torque(trial)
    assert ok, f"trial {trial_number} (seed={trial.seed}): {reason}"
