# --- collect.py ---
import subprocess, time, random, datetime, json, ast, sys, argparse
import utils
from schema import TrialData
import os

DIRS = {
    "server":    "../../RCSSServerMJ",
    "player":    "..",
    "obstacles": "../../obstacles",
}

FIELD = (-4.5, 4.5, -3.0, 3.0)  # xmin, xmax, ymin, ymax

RUN_CONFIG = {
    "A": { # test 1
        "stop_trigger": "robot_to_ball path:",
        "timeout":       60,
        "n_players":     1,
        "reset_ball":    False,   # ball position is set manually via monitor_client
        "log_metrics":  [],
    },
    "B": { # test 5
        "stop_trigger": "ball_stopped:",
        "timeout":       90,
        "n_players":     2,
        "reset_ball":    False,
        "log_metrics":  [
            "joint_angles",
            "joint_torques",
            "ball_stopped",
            "target_pos",
        ],
    },
    "C": { #test 6
        "stop_trigger": "ball_stopped:",
        "timeout":       90,
        "n_players":     1,
        "reset_ball":    False,
        "log_metrics":  [
            "joint_angles",
            "joint_torques",
            "ball_stopped",
            "target_pos",
        ],
    },
    "D": { #test 7-8
        "stop_trigger": "scored_at:",
        "timeout":      180,
        "n_players":     2,
        "reset_ball":    True,    # full game scenario, use default center position
       "log_metrics":  [
            "com_height",
            "com_z_vel",
            "com_x_vel",
            "velocity",
            "collision",
            "out_of_bounds",
            "latency_ms",
            "msg_sent",
            "msg_received",
            "scored_at",
        ],
    },
    "E": { #test 9
        "stop_trigger": "reached_ball",
        "timeout":       90,
        "n_players":     1,
        "reset_ball":    False,
       "log_metrics":  [
            "velocity",
            "solo",
        ],
    },
}


# ── Trial execution ────────────────────────────────────────────────────────────

def _spawn_player(number: int, start_pos: tuple, config: dict, dirs: dict) -> subprocess.Popen:
    """Spawns a single player process with the given number and start position."""
    return subprocess.Popen(
        ["hatch", "run", "python3", "run_player.py",
         "--host", "localhost", "--port", "60000",
         "-n", str(number), "-t", "Team",
         "--spawn-x",   str(start_pos[0]),
         "--spawn-y",   str(start_pos[1]),
         "--spawn-rot", "0"],
        cwd=dirs["player"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        start_new_session=True,
        env={
            **os.environ,
            "LOG_METRICS": ",".join(config["log_metrics"]),
        },
    )


def run_trial(run_id, trial_number, start_positions, ball_pos, obstacles, logger) -> TrialData:
    """
    start_positions is now a list of (x, y) tuples, one per player.
    Player numbers are assigned 1..n in order.
    Log lines are only collected from player 1 — it is always the passer.
    """
    config = RUN_CONFIG[run_id]
    seed   = random.randint(0, 99999)
    data   = TrialData(
        run_id          = run_id,
        trial_number    = trial_number,
        seed            = seed,
        timestamp       = datetime.datetime.now().isoformat(),
        start_position  = start_positions[0],   # primary player position
        ball_position   = ball_pos,
        obstacle_config = obstacles,
    )

    server         = None
    players        = []
    obstacle_procs = []

    try:
        # Start server
        server, _ = utils.popen_with_logged_output(
            ["hatch", "run", "rcssservermj", "--no-render"],
            # ["hatch", "run", "rcssservermj"],
            cwd=DIRS["server"], logger=logger, label="server", start_new_session=True,
            env={
                **os.environ,
                "RESET_BALL_ON_KICKOFF": "0" if not config["reset_ball"] else "1",
            },
        )
        time.sleep(3)

        # Start players — player 1 first, then teammates
        for number, start_pos in enumerate(start_positions, start=1):
            p = _spawn_player(number, start_pos, config, DIRS)
            players.append(p)
            logger.info(f"[runner] spawned player {number} at {start_pos}")
            time.sleep(1)   # stagger spawns so server registers each connection

        time.sleep(2)   # let all players finish connecting before kickoff

        # Start obstacles
        for i, (ox, oy) in enumerate(obstacles, start=1):
            p, _ = utils.popen_with_logged_output(
                ["python3", "run_obstacles.py",
                 "--number", str(i), "--x", str(ox), "--y", str(oy)],
                cwd=DIRS["obstacles"], logger=logger,
                label=f"obs_{i}", start_new_session=True,
            )
            obstacle_procs.append(p)
        time.sleep(1)

        # Kickoff
        try:
            if not config["reset_ball"]:
                subprocess.run(
                    ["python3", "monitor_client.py", "ball",
                    f'"(pos {ball_pos[0]} {ball_pos[1]} 0)"'],
                    cwd=DIRS["server"], timeout=2, capture_output=True,
                )
            time.sleep(4)
            subprocess.run(
                ["python3", "monitor_client.py", "kickOff", "Left"],
                cwd=DIRS["server"], timeout=2, capture_output=True,
            )
        except subprocess.TimeoutExpired:
            pass

        # Read output from player 1 only — it is always the passer and
        # the source of all logged metrics. Teammates run silently.
        start_time = time.time()
        while True:
            line = players[0].stdout.readline()

            if not line:
                if players[0].poll() is not None:
                    data.player_crashed = True
                    break
                continue

            line = line.rstrip("\n")
            logger.info(f"[player1] {line}")
            data.log_lines.append(line)
            _parse_line(line, data)

            if config["stop_trigger"] in line:
                break

            if time.time() - start_time > config["timeout"]:
                data.timed_out = True
                logger.warning(utils.color(
                    f"[run {run_id}] trial {trial_number} timed out "
                    f"after {config['timeout']}s", "yellow"
                ))
                break

    finally:
        for i, p in enumerate(obstacle_procs, 1):
            utils.terminate_process_tree(p, logger, f"obs_{i}")
        for i, p in enumerate(players, 1):
            utils.terminate_process_tree(p, logger, f"player_{i}")
        if server:
            utils.terminate_process_tree(server, logger, "server")

    return data


# ── Log line parser ────────────────────────────────────────────────────────────

def _parse_line(line: str, data: TrialData):
    """
    Extracts structured data from a single log line.
    Only lines matching the run's log_metrics will appear —
    the player is told what to emit via LOG_METRICS env var.

    Log line formats (emitted by player/simulation):
      [metric] velocity: 0.34
      [metric] com_height: 0.91
      [metric] com_z_vel: -0.02
      [metric] com_x_vel: 0.15
      [metric] joint_angles: {"hip_l": 12.3, "knee_l": -5.1, ...}
      [metric] joint_torques: {"hip_l": 45.2, "knee_l": 38.7, ...}
      [metric] latency_ms: 4.7
      [metric] collision at: 12.34
      [metric] out_of_bounds
      [metric] ball_stopped: (3.2, 1.1)
      [metric] target_pos: (3.0, 1.0)
      [metric] scored_at: 47.2
      [metric] msg_sent
      [metric] msg_received
    """
    try:
        if   "velocity:"      in line: data.velocities.append(float(line.split("velocity:")[1].strip()))
        elif "com_height:"    in line: data.com_heights.append(float(line.split("com_height:")[1].strip()))
        elif "com_z_vel:"     in line: data.com_z_velocities.append(float(line.split("com_z_vel:")[1].strip()))
        elif "com_x_vel:"     in line: data.com_x_velocities.append(float(line.split("com_x_vel:")[1].strip()))
        elif "latency_ms:"    in line: data.latencies_ms.append(float(line.split("latency_ms:")[1].strip()))
        elif "collision at:"  in line: data.collision_times.append(float(line.split("collision at:")[1].strip()))
        elif "out_of_bounds"  in line: data.out_of_bounds_steps += 1
        elif "scored_at:"     in line: data.time_to_score_seconds = float(line.split("scored_at:")[1].strip())
        elif "msg_sent"       in line: data.messages_sent += 1
        elif "msg_received"   in line: data.messages_received += 1
        elif "ball_stopped:"  in line: data.ball_final_pos  = tuple(ast.literal_eval(line.split("ball_stopped:")[1].strip()))
        elif "target_pos:"    in line: data.ball_target_pos = tuple(ast.literal_eval(line.split("target_pos:")[1].strip()))
        elif "joint_angles:"  in line: data.joint_angles.append(json.loads(line.split("joint_angles:")[1].strip()))
        elif "joint_torques:" in line: data.joint_torques.append(json.loads(line.split("joint_torques:")[1].strip()))
        data.total_steps += 1
    except (ValueError, IndexError, SyntaxError):
        pass


# ── Run configs ────────────────────────────────────────────────────────────────

def random_point():
    xmin, xmax, ymin, ymax = FIELD
    return (round(random.uniform(xmin, xmax), 3),
            round(random.uniform(ymin, ymax), 3))


def configs_for(run_id: str) -> list[tuple]:
    """
    Returns a list of (obstacles, start_positions, ball_pos) tuples.
    start_positions is always a list — one entry per player in n_players.

    A: 10 obstacle configs x 10 positions = 100 trials, 1 player
       robot only plans a path, does not move
    B: 100 random trials, 2 players
       player 1 walks to ball and passes to player 2
    C: 100 random trials, 1 player
       robot walks to ball and attempts a scoring kick
    D: 100 random trials, 2 players
       two robots execute full pass-and-score sequence
    E: 20 trials, fixed straight-on approach, 1 player
       robot walks directly to ball, velocity logged every timestep
    """
    n = RUN_CONFIG[run_id]["n_players"]

    if run_id == "A":
        obs_configs = [[random_point() for _ in range(3)] for _ in range(10)]
        starts      = [random_point() for _ in range(10)]
        balls       = [random_point() for _ in range(10)]
        return [
            (obs, [starts[i % 10]], balls[i % 10])
            for obs in obs_configs
            for i in range(10)
        ]

    if run_id in ("B", "C", "D"):
        return [
            (
                [random_point() for _ in range(3)],
                [random_point() for _ in range(n)],   # one start per player
                random_point(),
            )
            for _ in range(100)
        ]

    if run_id == "E":
        return [
            ([], [(-3.0, 0.0)], (3.0, 0.0))
            for _ in range(20)
        ]

    raise ValueError(f"Unknown run_id: {run_id}")


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Run simulation trials and save data to disk.")
    parser.add_argument(
        "runs", nargs="*", default=["A", "B", "C", "D", "E"],
        help="Which runs to collect (default: all). Example: python collect.py A D"
    )
    args = parser.parse_args()

    logger = utils.setup_test_logging("collect")

    for run_id in args.runs:
        if run_id not in RUN_CONFIG:
            logger.error(f"Unknown run_id '{run_id}'. Valid options: {list(RUN_CONFIG.keys())}")
            continue

        trial_configs = configs_for(run_id)
        n_players = RUN_CONFIG[run_id]["n_players"]
        logger.info(utils.color(
            f"\n{'='*50}\nCollecting Run {run_id} "
            f"({len(trial_configs)} trials, "
            f"{n_players} player(s), "
            f"stop={RUN_CONFIG[run_id]['stop_trigger']!r}, "
            f"timeout={RUN_CONFIG[run_id]['timeout']}s)\n{'='*50}",
            "blue"
        ))

        for i, (obstacles, start_positions, ball) in enumerate(trial_configs, start=1):
            logger.info(f"[run {run_id}] trial {i}/{len(trial_configs)} "
                        f"starts={start_positions} ball={ball} obstacles={obstacles}")

            trial = run_trial(run_id, i, start_positions, ball, obstacles, logger)
            trial.save()

            if trial.player_crashed:
                status, c = "CRASH", "red"
            elif trial.timed_out:
                status, c = "TIMEOUT", "yellow"
            else:
                status, c = "SAVED", "green"

            logger.info(utils.color(
                f"[run {run_id}] trial {i} {status} (seed={trial.seed})", c
            ))

        logger.info(utils.color(f"[run {run_id}] Done.\n", "blue"))


if __name__ == "__main__":
    main()