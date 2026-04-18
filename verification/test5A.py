"""
Collect data first: `python collect.py B` (creates data/run_B_*.json).
Run from this directory: `pytest test5A.py -v`
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pytest

from evaluators import check_kick_accuracy, kick_error

ERROR_THRESHOLD = 0.5

_TRIAL_NUMBERS = list(range(1, 101))
_RECORDED_ERRORS: dict[int, float] = {}
_OUTPUT_DIR = Path(__file__).resolve().parent / "output"
_PLOT_PATH = _OUTPUT_DIR / "test5A_error_plot.png"


@pytest.fixture(scope="module", autouse=True)
def _report_recorded_errors(request):
    """
    Save kick-error plot and print aggregate stats after all test cases run.
    """
    _RECORDED_ERRORS.clear()
    yield

    if not _RECORDED_ERRORS:
        return

    terminal = request.config.pluginmanager.getplugin("terminalreporter")
    if terminal is None:
        return

    trials = sorted(_RECORDED_ERRORS.items())
    errors = [err for _, err in trials]
    avg_error = sum(errors) / len(errors)

    _OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(12, 4))
    bins = min(20, max(5, len(errors) // 5))
    ax.hist(errors, bins=bins, color="steelblue", edgecolor="black", alpha=0.8)
    ax.axvline(avg_error, color="darkorange", linestyle="--", linewidth=2, label=f"avg={avg_error:.3f} m")
    ax.axvline(ERROR_THRESHOLD, color="crimson", linestyle=":", linewidth=2, label=f"threshold={ERROR_THRESHOLD:.3f} m")
    ax.set_title("Test 5A Error Histogram")
    ax.set_xlabel("Error (m)")
    ax.set_ylabel("Trial count")
    ax.grid(axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(_PLOT_PATH, dpi=150)
    plt.close(fig)

    terminal.write_line("")
    terminal.write_sep("=", "test5A kick-error report")
    terminal.write_line(
        f"Recorded {len(errors)} trials | Average error: {avg_error:.3f} m | "
        f"Threshold: {ERROR_THRESHOLD:.3f} m"
    )
    terminal.write_line(f"Saved matplotlib plot: {_PLOT_PATH}")


@pytest.mark.verification_suite("5A")
@pytest.mark.parametrize(
    "trial_number",
    _TRIAL_NUMBERS,
    ids=[f"trial_{n}" for n in _TRIAL_NUMBERS],
)
def test5A(run_b, trial_number: int):
    """Test 5A: passing accuracy."""
    trial = next((t for t in run_b if t.trial_number == trial_number), None)
    assert trial is not None, (
        f"no trial with trial_number={trial_number} in run B data"
        f"(expected {len(_TRIAL_NUMBERS)} files under data/)"
    )

    err = kick_error(trial)
    assert err is not None, f"trial {trial_number} (seed={trial.seed}): ball position not logged"
    _RECORDED_ERRORS[trial_number] = err

    ok, reason = check_kick_accuracy(trial, ERROR_THRESHOLD)
    assert ok, f"trial {trial_number} (seed={trial.seed}): {reason}"
