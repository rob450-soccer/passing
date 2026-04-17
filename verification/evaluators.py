# --- evaluators.py ---
# All evaluation logic in one file. Pure functions, no I/O.

import math, ast
from schema import TrialData

ROBOT_RADIUS    = 0.23
MIN_SAFE_DIST   = 2 * ROBOT_RADIUS
MAX_TORQUE      = 100.0
JOINT_MARGIN    = 1.0
PASS_THRESHOLD  = 0.5
SCORE_THRESHOLD = 2.0
MAX_VELOCITY    = 0.5
MAX_COLLISIONS_PER_MIN = 1.3
MAX_OOB_FRACTION       = 0.01
MAX_LATENCY_MS         = 20.0
MAX_PASS_FALLS_PER_MIN = 0.15
MAX_LOCO_FALLS_PER_MIN = 0.82
MAX_COMM_DROP_RATE     = 0.05
GRAVITY                = 9.81


# ── geometry ──────────────────────────────────────────────────────────────────

def point_to_segment_dist(p, a, b) -> float:
    px, py = p;  ax, ay = a;  bx, by = b
    l2 = (bx-ax)**2 + (by-ay)**2
    if l2 == 0:
        return math.hypot(px-ax, py-ay)
    t = max(0, min(1, ((px-ax)*(bx-ax) + (py-ay)*(by-ay)) / l2))
    return math.hypot(px - (ax + t*(bx-ax)), py - (ay + t*(by-ay)))

def path_hits_obstacle(path, obstacles, min_dist=MIN_SAFE_DIST) -> bool:
    if len(path) < 2:
        return False
    for a, b in zip(path, path[1:]):
        for obs in obstacles:
            if point_to_segment_dist(obs, a, b) < min_dist:
                return True
    return False


# ── log parsing ───────────────────────────────────────────────────────────────

def parse_path(log_line: str) -> list[tuple]:
    try:
        return [(float(c[0]), float(c[1]))
                for c in ast.literal_eval(log_line.split("path:")[1].strip())]
    except (IndexError, ValueError, SyntaxError):
        return []

def parse_scale(log_line: str) -> float | None:
    try:
        return float(log_line.split("scale ")[1].strip())
    except (IndexError, ValueError):
        return None

def extract_paths_and_scale(log_lines: list[str]):
    paths = {}
    scale = None
    for line in log_lines:
        if "grid world created with scale" in line:
            scale = parse_scale(line)
        for trigger, idx in [("robot_to_ball path:", 0),
                              ("dribble path:", 1),
                              ("robot_to_receive path:",  2)]:
            if trigger in line:
                paths[idx] = parse_path(line)
    return [paths[i] for i in sorted(paths)], scale


# ── Test 1 ────────────────────────────────────────────────────────────────────

def check_paths(trial: TrialData) -> tuple[bool, str]:
    """Test 1A — no path collides with an obstacle."""
    paths, scale = extract_paths_and_scale(trial.log_lines)

    if not paths:   return False, "no paths logged"
    if not scale:   return False, "grid scale not logged"

    for i, path in enumerate(paths):
        path_m = [(x/scale, y/scale) for x, y in path]
        if path_hits_obstacle(path_m, trial.obstacle_config):
            return False, f"path {i} intersects obstacle"

    return True, ""


# ── Tests 5 & 6 ───────────────────────────────────────────────────────────────

def check_kick(trial: TrialData, error_threshold: float,
               joint_limits: dict) -> tuple[bool, str]:
    """Tests 5A/6A (accuracy), 5B/6B (torque), 5C/6C (joint limits)."""
    if not trial.ball_final_pos or not trial.ball_target_pos:
        return False, "ball position not logged"

    err = math.hypot(trial.ball_final_pos[0] - trial.ball_target_pos[0],
                     trial.ball_final_pos[1] - trial.ball_target_pos[1])
    if err > error_threshold:
        return False, f"kick error {err:.2f}m > {error_threshold}m"

    if trial.joint_torques:
        max_t = max(abs(v) for step in trial.joint_torques for v in step.values())
        if max_t > MAX_TORQUE:
            return False, f"torque {max_t:.1f} Nm > {MAX_TORQUE} Nm"

    for step in trial.joint_angles:
        for joint, angle in step.items():
            lo, hi = joint_limits.get(joint, (-180.0, 180.0))
            if not (lo - JOINT_MARGIN <= angle <= hi + JOINT_MARGIN):
                return False, f"joint {joint} angle {angle:.1f}° out of limits"

    return True, ""


# ── Test 8 ────────────────────────────────────────────────────────────────────

def check_comm(trial: TrialData) -> tuple[bool, str]:
    """Test 8B — message drop rate < 5%."""
    if trial.messages_sent == 0:
        return False, "no messages sent"
    drop = 1 - trial.messages_received / trial.messages_sent
    if drop > MAX_COMM_DROP_RATE:
        return False, f"drop rate {drop:.1%} > {MAX_COMM_DROP_RATE:.1%}"
    return True, ""

def check_falls(trial: TrialData, duration_min: float) -> tuple[bool, str]:
    """Test 8C — fall rate using Koolen zcrit metric."""
    falls = in_fall = 0
    for z, zd, xd in zip(trial.com_heights, trial.com_z_velocities, trial.com_x_velocities):
        d = z**2 - xd**2 / (2 * GRAVITY)
        zcrit = z + math.sqrt(max(d, 0)) if d >= 0 else -1.0
        if zcrit <= 0 and not in_fall:
            falls += 1;  in_fall = True
        elif zcrit > 0:
            in_fall = False
    rate = falls / duration_min
    if rate > MAX_LOCO_FALLS_PER_MIN:
        return False, f"fall rate {rate:.2f}/min > {MAX_LOCO_FALLS_PER_MIN}"
    return True, ""

def check_locomotion(trial: TrialData, duration_min: float) -> tuple[bool, str]:
    """Tests 8D (collisions), 8E (OOB), 9A (velocity)."""
    if trial.velocities and max(trial.velocities) > MAX_VELOCITY:
        return False, f"max velocity {max(trial.velocities):.2f} m/s > {MAX_VELOCITY}"

    cpm = len(trial.collision_times) / duration_min
    if cpm >= MAX_COLLISIONS_PER_MIN:
        return False, f"collision rate {cpm:.2f}/min >= {MAX_COLLISIONS_PER_MIN}"

    if trial.total_steps:
        oob = trial.out_of_bounds_steps / trial.total_steps
        if oob > MAX_OOB_FRACTION:
            return False, f"OOB fraction {oob:.1%} > {MAX_OOB_FRACTION:.1%}"

    return True, ""

def check_latency(trial: TrialData) -> tuple[bool, str]:
    """Test 8F — max action latency < 20ms."""
    if not trial.latencies_ms:
        return True, ""
    mx = max(trial.latencies_ms)
    if mx > MAX_LATENCY_MS:
        return False, f"max latency {mx:.1f}ms > {MAX_LATENCY_MS}ms"
    return True, ""