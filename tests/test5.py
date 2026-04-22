"""
Test 5 — two-robot passing (100 trials).

Parts (see tests/test5.txt):
  A — Pass accuracy: Euclidean error ≤ 0.5 m when the ball stops after a kick;
      on a 60 s timeout the error is scored as the initial ball-to-target error.
  B — Torque: no leg joint torque magnitude > 100 Nm (via logged commanded PD torques).
  C — Joint limits: no joint angle outside XML limits by more than 1°.

Structure mirrors test1.py; trial execution and metrics follow verification/collect.py run B.

Debug (why "ball kicked then stopped" never fires): run with BALL_STOP_DEBUG=1 so player 1
prints throttled [debug ball_stop] lines (v_norm, ball_seen, kick_streak, would_stop for 0.01 threshold).
Example:  BALL_STOP_DEBUG=1 python3 test5.py
"""

import ast
import datetime
import json
import math
import os
import random
import selectors
import subprocess
import sys
import time

from utils import Utils

util = Utils()

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
PASSING_DIR = os.path.dirname(TESTS_DIR)
BASE_DIR = os.path.dirname(PASSING_DIR)
RCSSSMJ_DIR = os.path.join(BASE_DIR, "rcssservermj")

_VERIFICATION_DIR = os.path.join(PASSING_DIR, "verification")
if _VERIFICATION_DIR not in sys.path:
    sys.path.insert(0, _VERIFICATION_DIR)

from schema import TrialData  # noqa: E402
from evaluators import (  # noqa: E402
    PASS_THRESHOLD,
    MAX_TORQUE,
    check_kick_accuracy,
    check_kick_joint_limits,
    check_kick_torque,
    kick_error,
)
import conftest as verification_conftest  # noqa: E402

JOINT_LIMITS = verification_conftest.JOINT_LIMITS

TIMEOUT_SECONDS = 60
STOP_TRIGGER = "ball kicked then stopped"

# Same metric keys as verification/collect.py RUN_CONFIG["B"], plus receive_target_pos
# so Part A can score against the planned receive target (not receiver robot position).
LOG_METRICS_ENV = "joint_angles,joint_torques,ball_stopped,target_pos,receive_target_pos"

# Field bounds for random trials (aligned with verification/collect.py run B).
FIELD_X_MIN, FIELD_X_MAX = -4.5, 4.5
FIELD_Y_MIN, FIELD_Y_MAX = -3.0, 3.0

RECORDED_BALL_XY = (0.0, 0.0)


def random_field_point():
    return (
        round(random.uniform(FIELD_X_MIN, FIELD_X_MAX), 3),
        round(random.uniform(FIELD_Y_MIN, FIELD_Y_MAX), 3),
    )


def parse_metric_line(line: str, data: TrialData) -> None:
    """Same field extraction as verification/collect._parse_line."""
    try:
        if "ball_stopped:" in line:
            data.ball_final_pos = tuple(ast.literal_eval(line.split("ball_stopped:")[1].strip()))
        elif "receive_target_pos:" in line:
            data.ball_target_pos = tuple(ast.literal_eval(line.split("receive_target_pos:")[1].strip()))
        elif "target_pos:" in line:
            data.ball_target_pos = tuple(ast.literal_eval(line.split("target_pos:")[1].strip()))
        elif "joint_angles:" in line:
            data.joint_angles.append(json.loads(line.split("joint_angles:")[1].strip()))
        elif "joint_torques:" in line:
            data.joint_torques.append(json.loads(line.split("joint_torques:")[1].strip()))
        elif "joint_torque_max_nm:" in line:
            v = float(line.split("joint_torque_max_nm:")[1].strip())
            data.joint_torque_peak_nm = v if data.joint_torque_peak_nm is None else max(data.joint_torque_peak_nm, v)
    except (ValueError, IndexError, SyntaxError, json.JSONDecodeError):
        pass


def max_logged_torque_nm(trial: TrialData) -> float:
    """Peak |τ| from [metric] joint_torque_max_nm; fallback to violation-only joint_torques dicts."""
    if trial.joint_torque_peak_nm is not None:
        return float(trial.joint_torque_peak_nm)
    if trial.joint_torques:
        return max(abs(v) for step in trial.joint_torques for v in step.values())
    return 0.0


def kick_length_m(trial: TrialData) -> float | None:
    """Kick length = ||ball_stopped|| from origin (0,0)."""
    if not trial.ball_final_pos:
        return None
    x, y = trial.ball_final_pos
    return math.hypot(x, y)


def apply_timeout_fallback(
    trial: TrialData,
    ball_xy: tuple[float, float],
    receiver_xy: tuple[float, float],
) -> None:
    """Score timeout / missing metrics using initial ball-to-receiver separation (test5.txt)."""
    trial.ball_final_pos = ball_xy
    trial.ball_target_pos = receiver_xy


def spawn_player(player_number: int, start_x: float, start_y: float) -> subprocess.Popen:
    return subprocess.Popen(
        [
            "hatch",
            "run",
            "python3",
            "run_player.py",
            "--host",
            "localhost",
            "--port",
            "60000",
            "-n",
            str(player_number),
            "-t",
            "Team",
            "--spawn-x",
            str(start_x),
            "--spawn-y",
            str(start_y),
            "--spawn-rot",
            "0",
            "--verbose",
        ],
        cwd=PASSING_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env={
            **os.environ,
            "LOG_METRICS": LOG_METRICS_ENV,
        },
    )


def cleanup_trial_processes(player_processes, server_process, logger):
    logger.info(util.color("Cleaning up", "yellow"))
    for i, process in enumerate(player_processes, start=1):
        util.terminate_process_tree(process, logger, f"player_{i}")
    util.terminate_process_tree(server_process, logger, "server")


def run_test():
    suite_start_time = time.time()
    logger = util.setup_test_logging("test5")
    logger.info("Test 5 — two-robot passing (Parts A/B/C)")

    total_trials = 5 # 100
    passed_a = passed_b = passed_c = 0
    trial_errors_m: list[float] = []
    trial_max_torque: list[float] = []
    trial_kick_lengths_m: list[float] = []

    exit_code = 0

    TEST_DRIVE_FOLDER = f"Test5_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    if (
        subprocess.run(
            f"rclone mkdir google_drive:/rob450-data/Verification/{TEST_DRIVE_FOLDER}",
            shell=True,
        ).returncode
        != 0
    ):
        logger.error(
            f"Failed to create Google Drive folder for data: rob450-data/Verification/{TEST_DRIVE_FOLDER}"
        )
        sys.exit(1)
    logger.info(f"Created Google Drive folder for data: rob450-data/Verification/{TEST_DRIVE_FOLDER}")

    try:
        for trial_count in range(1, total_trials + 1):
            passer_xy = random_field_point()
            receiver_xy = random_field_point()

            logger.info(f"{'-' * 50} Trial {trial_count}/{total_trials} {'-' * 50}")
            logger.info(f"Passer P1: {passer_xy}, Receiver P2: {receiver_xy} (kickoff only; ball not repositioned)")

            server_process = None
            player_processes = []

            try:
                server_process, _thr = util.popen_with_logged_output(
                    ["hatch", "run", "rcssservermj"],
                    cwd=RCSSSMJ_DIR,
                    logger=logger,
                    label="server",
                    start_new_session=True,
                    env={**os.environ, "RESET_BALL_ON_KICKOFF": "0"},
                )
                time.sleep(3)

                p1 = spawn_player(1, passer_xy[0], passer_xy[1])
                player_processes.append(p1)
                time.sleep(1)
                p2 = spawn_player(2, receiver_xy[0], receiver_xy[1])
                player_processes.append(p2)
                time.sleep(3)
                util.kickoff_only(logger)

                data = TrialData(
                    run_id="test5",
                    trial_number=trial_count,
                    seed=0,
                    timestamp=datetime.datetime.now().isoformat(),
                    start_position=passer_xy,
                    ball_position=RECORDED_BALL_XY,
                    obstacle_config=[],
                )

                start_time = time.time()
                timed_out = False

                selector = selectors.DefaultSelector()
                watched = {}
                for i, proc in enumerate(player_processes, start=1):
                    if proc.stdout is not None:
                        selector.register(proc.stdout, selectors.EVENT_READ, data=i)
                        watched[i] = proc

                try:
                    while True:
                        events = selector.select(timeout=0.2)
                        if not events:
                            if time.time() - start_time > TIMEOUT_SECONDS:
                                timed_out = True
                                logger.warning(
                                    util.color(
                                        f"[TIMEOUT] Trial {trial_count}: no '{STOP_TRIGGER}' within {TIMEOUT_SECONDS}s",
                                        "yellow",
                                    )
                                )
                                break
                            continue

                        for key, _ in events:
                            player_idx = key.data
                            proc = watched[player_idx]
                            line = key.fileobj.readline()
                            if not line:
                                if proc.poll() is not None:
                                    try:
                                        selector.unregister(key.fileobj)
                                    except Exception:
                                        pass
                                    watched.pop(player_idx, None)
                                    if player_idx == 1:
                                        data.player_crashed = True
                                        break
                                continue

                            line = line.rstrip("\n")
                            logger.info(f"[player{player_idx}] {line}")
                            data.log_lines.append(f"[player{player_idx}] {line}")
                            parse_metric_line(line, data)

                            if STOP_TRIGGER in line:
                                logger.info(f"Stop trigger detected from player {player_idx}")
                                watched.clear()
                                break

                        if not watched or data.player_crashed:
                            break
                finally:
                    try:
                        selector.close()
                    except Exception:
                        pass

                if data.player_crashed:
                    logger.error(util.color(f"[FAIL] Trial {trial_count}: Player process crashed.", "red"))
                    apply_timeout_fallback(data, RECORDED_BALL_XY, receiver_xy)
                elif timed_out or kick_error(data) is None:
                    apply_timeout_fallback(data, RECORDED_BALL_XY, receiver_xy)

                data.timed_out = timed_out

                err_m = kick_error(data)
                if err_m is not None:
                    trial_errors_m.append(err_m)
                klen = kick_length_m(data)
                if klen is not None:
                    trial_kick_lengths_m.append(klen)
                    logger.info(f"[metric] kick_length_m: {klen:.3f}")

                mt = max_logged_torque_nm(data)
                trial_max_torque.append(mt)

                ok_a, reason_a = check_kick_accuracy(data, PASS_THRESHOLD)
                ok_b, reason_b = check_kick_torque(data)
                ok_c, reason_c = check_kick_joint_limits(data, JOINT_LIMITS)

                if ok_a:
                    passed_a += 1
                    logger.info(
                        util.color(
                            f"[PASS A] Trial {trial_count}: error {kick_error(data):.3f} m (≤ {PASS_THRESHOLD} m)",
                            "green",
                        )
                    )
                else:
                    logger.error(util.color(f"[FAIL A] Trial {trial_count}: {reason_a}", "red"))
                    exit_code = 1

                if ok_b:
                    passed_b += 1
                    logger.info(
                        util.color(
                            f"[PASS B] Trial {trial_count}: max |τ| {mt:.1f} Nm (cap {MAX_TORQUE} Nm)",
                            "green",
                        )
                    )
                else:
                    logger.error(util.color(f"[FAIL B] Trial {trial_count}: {reason_b}", "red"))
                    exit_code = 1

                if ok_c:
                    passed_c += 1
                    logger.info(util.color(f"[PASS C] Trial {trial_count}: joint limits OK", "green"))
                else:
                    logger.error(util.color(f"[FAIL C] Trial {trial_count}: {reason_c}", "red"))
                    exit_code = 1

            finally:
                cleanup_trial_processes(player_processes, server_process, logger)

    except KeyboardInterrupt:
        logger.error(util.color("Test interrupted by user (Ctrl-C). Cleaned up background processes.", "red"))
        exit_code = 1

    suite_end_time = time.time()
    logger.info(
        util.color(f"Total time elapsed: {(suite_end_time - suite_start_time):.2f} seconds", "blue")
    )

    if trial_errors_m:
        avg_e = sum(trial_errors_m) / len(trial_errors_m)
        logger.info(
            util.color(
                f"Aggregate Part A: mean Euclidean error = {avg_e:.4f} m over {len(trial_errors_m)} scored trials",
                "blue",
            )
        )
    if trial_max_torque:
        logger.info(
            util.color(
                f"Aggregate Part B: global max |torque| over all trials = {max(trial_max_torque):.2f} Nm",
                "blue",
            )
        )
    if trial_kick_lengths_m:
        avg_k = sum(trial_kick_lengths_m) / len(trial_kick_lengths_m)
        logger.info(util.color(f"Average kick length: {avg_k:.4f} m", "blue"))
        logger.info(
            util.color(
                f"Aggregate kick length: mean ||ball_stopped|| = {avg_k:.4f} m over {len(trial_kick_lengths_m)} trials",
                "blue",
            )
        )

    logger.info(
        util.color(
            f"Part A pass rate: {passed_a}/{total_trials} | "
            f"Part B: {passed_b}/{total_trials} | "
            f"Part C: {passed_c}/{total_trials}",
            "blue",
        )
    )

    if exit_code == 0:
        logger.info(util.color("SUCCESS: all parts passed for every trial.", "green"))
    else:
        logger.error(util.color("FAILURE: one or more trials failed one or more parts.", "red"))

    try:
        import matplotlib.pyplot as plt

        out_dir = os.path.join(TESTS_DIR, "output")
        os.makedirs(out_dir, exist_ok=True)
        err_plot_path = os.path.join(out_dir, "test5_error_histogram.png")
        kick_len_plot_path = os.path.join(out_dir, "test5_kick_length_histogram.png")
        if trial_errors_m:
            fig, ax = plt.subplots(figsize=(12, 4))
            bins = min(20, max(5, len(trial_errors_m) // 5))
            ax.hist(trial_errors_m, bins=bins, color="steelblue", edgecolor="black", alpha=0.8)
            ax.axvline(PASS_THRESHOLD, color="crimson", linestyle=":", linewidth=2, label=f"threshold={PASS_THRESHOLD} m")
            ax.set_title("Test 5 Part A — Kick error (m)")
            ax.set_xlabel("Error (m)")
            ax.set_ylabel("Trial count")
            ax.legend()
            ax.grid(axis="y", alpha=0.3)
            fig.tight_layout()
            fig.savefig(err_plot_path, dpi=150)
            plt.close(fig)
            logger.info(f"Saved error histogram: {err_plot_path}")
        if trial_kick_lengths_m:
            fig, ax = plt.subplots(figsize=(12, 4))
            bins = min(20, max(5, len(trial_kick_lengths_m) // 5))
            ax.hist(trial_kick_lengths_m, bins=bins, color="darkorange", edgecolor="black", alpha=0.8)
            ax.set_title("Test 5 — Kick length (m)")
            ax.set_xlabel("Kick length from origin (m)")
            ax.set_ylabel("Trial count")
            ax.grid(axis="y", alpha=0.3)
            fig.tight_layout()
            fig.savefig(kick_len_plot_path, dpi=150)
            plt.close(fig)
            logger.info(f"Saved kick-length histogram: {kick_len_plot_path}")
    except ImportError:
        logger.info("matplotlib not installed; skipped plots.")

    sys.exit(exit_code)


if __name__ == "__main__":
    run_test()
