# --- conftest.py ---
# Session-scoped fixtures so data loads once per pytest run.

import pytest
import re
from collections import defaultdict
from schema import TrialData

@pytest.fixture(scope="session")
def run_a(): return TrialData.load_run("A")

@pytest.fixture(scope="session")
def run_b(): return TrialData.load_run("B")

@pytest.fixture(scope="session")
def run_c(): return TrialData.load_run("C")

@pytest.fixture(scope="session")
def run_d(): return TrialData.load_run("D")

@pytest.fixture(scope="session")
def run_e(): return TrialData.load_run("E")

JOINT_LIMITS = {f"joint_{i}": (-90.0, 90.0) for i in range(23)}
DURATION_MIN = 1.0

_PARAM_ID_RE = re.compile(r"\[([^\]]+)\]\s*$")

# Populated during the run via ``pytest_runtest_logreport``; reset each session.
_summary: dict[str, dict] | None = None

def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "verification_suite(suite_id): group trial-style parametrized tests; "
        "each suite gets an extra pass/fail summary after the run",
    )


def pytest_collection_modifyitems(session, config, items):
    """
    Copy ``verification_suite`` mark args onto ``item.user_properties`` so they
    survive onto ``TestReport`` (``report.keywords`` only stores marker names).
    """
    for item in items:
        mk = item.get_closest_marker("verification_suite")
        if mk is None:
            continue
        args = mk.args or ()
        kwargs = mk.kwargs or {}
        suite_id = str(args[0]) if args else str(kwargs.get("suite_id", ""))
        if suite_id:
            item.user_properties.append(("verification_suite_id", suite_id))


def _verification_suite_id(report) -> str | None:
    """Return suite_id stamped during collection, or None."""
    for name, val in report.user_properties or []:
        if name == "verification_suite_id":
            return str(val)
    return None


def pytest_sessionstart(session):
    """Reset per-session counters (safe for multiple sessions in one process)."""
    global _summary
    _summary = defaultdict(
        lambda: {"passed": 0, "failed": 0, "skipped": 0, "failed_cases": []}
    )


def pytest_runtest_logreport(report):
    """
    Count call-phase outcomes for tests marked ``verification_suite``.
    ``terminalreporter.stats`` reports often omit mark metadata on older pytest,
    so we aggregate here instead of only in ``pytest_terminal_summary``.
    """
    if report.when != "call" or _summary is None:
        return
    suite_id = _verification_suite_id(report)
    if suite_id is None:
        return

    bucket = _summary[suite_id]
    if report.outcome == "passed":
        bucket["passed"] += 1
    elif report.outcome == "failed":
        bucket["failed"] += 1
        m = _PARAM_ID_RE.search(report.nodeid)
        bucket["failed_cases"].append(
            m.group(1) if m else report.nodeid.split("::")[-1]
        )
    elif report.outcome == "skipped":
        bucket["skipped"] += 1


def pytest_terminal_summary(terminalreporter, exitstatus, config):
    """Per-suite lines for any test marked with ``@pytest.mark.verification_suite(...)``."""
    if not _summary:
        return

    for suite_id in sorted(_summary.keys()):
        b = _summary[suite_id]
        total = b["passed"] + b["failed"] + b["skipped"]
        if total == 0:
            continue

        terminalreporter.write_line("\n")
        terminalreporter.write_sep(
            "=",
            f"verification_suite {suite_id!r}: "
            f"{b['passed']} passed, {b['failed']} failed, {b['skipped']} skipped "
            f"of {total}",
        )
        if b["failed_cases"]:
            terminalreporter.write_line(
                "Failed cases: " + ", ".join(b["failed_cases"])
            )