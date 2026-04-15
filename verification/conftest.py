# --- conftest.py ---
# Session-scoped fixtures so data loads once per pytest run.

import pytest
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