# Verification

This folder contains the tooling used to collect and evaluate passing scenario data in simulation.

## Files

- `collect.py` - runs trial batches for configured runs (`A` through `E`), launches server/players/obstacles, parses metric log lines, and saves trial JSON files.
- `schema.py` - defines the `TrialData` dataclass and load/save helpers for persisted trial data.
- `evaluators.py` - pure evaluation helpers for path safety, kick quality, communication reliability, locomotion limits, latency, and fall detection.
- `conftest.py` - pytest configuration: session fixtures (`run_a` … `run_e`), shared constants (`JOINT_LIMITS`, `DURATION_MIN`), the registered `verification_suite` marker, and hooks that print a per-suite pass/fail summary after the run for marked tests.
- `test1.py` - offline checks for run `A` using `evaluators.check_paths` (100 parametrized trials; see Pytest below).
- `utils.py` - logging and process-management utilities used by collection scripts.

## Prerequisites

From the implementation in `collect.py`, collection assumes:

- `hatch` is installed and available in `PATH`
- sibling directories exist relative to this folder:
  - simulation server: **`../../RCSSServerMJ` or `../../rcssservermj`** (same layout; the collector uses whichever directory exists first)
  - `..` (player project containing `run_player.py`)
  - `../../obstacles` (obstacle runner containing `run_obstacles.py`)
- Python 3 environment with dependencies used by the player/server projects

## Run Profiles

`collect.py` defines five run types:

- `A` - path planning checks (1 player); `stop_trigger` is a **list** of log substrings that must all appear before the trial ends; metrics include `solo` via `LOG_METRICS`
- `B` - pass to teammate (2 players)
- `C` - single-player scoring kick (1 player)
- `D` - full pass-and-score flow (2 players)
- `E` - straight-line locomotion trials (1 player)

Each profile has its own timeout, stop trigger (string or list of strings), and selected metrics (`LOG_METRICS` env var) that are emitted by player logs and parsed into `TrialData`.

## Collecting Data

Run from this directory:

```bash
python3 collect.py
```

This runs all profiles (`A B C D E`).

To run specific profiles:

```bash
python3 collect.py A D
```

Outputs:

- `data/run_<RUN_ID>_trial_<NNN>.json` - saved trial records
- `logs/collect_<timestamp>.log` - collector and subprocess logs

## Data Format

Each trial file stores:

- metadata (`run_id`, `trial_number`, `seed`, timestamp, start/ball/obstacle positions)
- raw log lines (`log_lines`)
- parsed timeseries metrics (velocity, COM data, torques, joint angles, latency, collisions, etc.)
- end-of-trial outcomes (ball final/target position, score time, message counts, timeout/crash flags)

`TrialData.load_run("<ID>")` loads all persisted trials for a run (default directory `data/`, relative to the current working directory).

Trial counts produced by `collect.py` today: run `A`, `B`, `C`, and `D` each schedule **100** trials; run `E` schedules **20**.

## Pytest (offline checks on collected data)

1. Collect trials (example for run `A` only):

   ```bash
   python3 collect.py A
   ```

2. Install pytest if needed (`pip install pytest`, or add it to your Hatch/env).

3. Run tests from **this** directory so imports resolve (`schema`, `evaluators`, `conftest`) and `data/` is found:

   ```bash
   cd passing/verification
   pytest test1.py -v
   ```

`test1.py` defines `test1A`, parametrized over all **100** run-`A` trials, calling `evaluators.check_paths`. Tests that should appear in the extra summary line are marked with **`@pytest.mark.verification_suite("<id>")`** (for example suite `"1A"`). The hook copies that id onto each item’s `user_properties` during collection so it is available on test reports, then `pytest_runtest_logreport` aggregates outcomes and `pytest_terminal_summary` prints one block per suite id (totals plus `Failed cases:` for parametrized ids).

Session-scoped fixtures in `conftest.py` (`run_a`, `run_b`, …) call `TrialData.load_run(...)` once per session and return a `list[TrialData]`. Inject a fixture by name in your test, then call helpers from `evaluators.py` on each `TrialData`.

## Evaluation Helpers

`evaluators.py` provides reusable checks, for example:

- `check_paths(...)`
- `check_kick(...)`
- `check_comm(...)`
- `check_falls(...)`
- `check_locomotion(...)`
- `check_latency(...)`

These are pure functions (no file I/O), intended to be called by tests or analysis scripts that consume `TrialData`.

## Notes

- Collection parses metrics from player 1 output only.
- Unknown run IDs are skipped with an error log.
- If no data exists for a run, `TrialData.load_run(...)` raises with a command hint.
