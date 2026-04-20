import json
import logging
import math
import os
import threading
import random
import time
import xml.etree.ElementTree as ET
from enum import Enum
from pathlib import Path

import numpy as np
from mujococodebase.utils.math_ops import MathOps
from mujococodebase.world.play_mode import PlayModeEnum, PlayModeGroupEnum
from mujococodebase.world.grid_world import GridWorld
from mujococodebase.planning.planning import ana_theta_star as planner
from mujococodebase.planning.path_follower import PathFollower
from mujococodebase.world.other_robot import OtherRobot
from mujococodebase.planning.path_viz_emitter import emit as _viz_emit


logger = logging.getLogger(__file__)

ROBOT_RADIUS = 0.23 # meters

LOG_METRICS = set(os.environ.get("LOG_METRICS", "").split(","))
SIM_TIMESTEP = 0.02

# Set BALL_STOP_DEBUG=1 in the environment for throttled stdout lines when diagnosing
# tests 5/6 "ball kicked then stopped" / [metric] ball_stopped.
def _ball_stop_debug_enabled() -> bool:
    return os.environ.get("BALL_STOP_DEBUG", "").lower() in ("1", "true", "yes")

# Match passing/verification/evaluators.py (tests 5B/5C, 6B/6C).
MAX_KICK_TORQUE_NM = 100.0
JOINT_LIMIT_MARGIN_DEG = 1.0

# Tests 5/6: XY speed below this counts as "stopped". |v|<=0.01 missed slow rolls and jitter;
# Z noise from sim is ignored by using horizontal norm only.
_BALL_STOPPED_SPEED_XY_THRESH = 0.15

# Env keys that enable logging of PD torques implied by the last motor command
# (same law as rcssservermj DefaultActionParser + Simulation.ctrl_motor).
_LOG_COMMANDED_PD_TORQUE_KEYS = frozenset({"joint_commanded_pd_torques", "joint_torques"})

_joint_limits_cache: dict[str, tuple[float, float]] | None = None


class State(Enum):
    BEAMING = 0
    GETTING_UP = 1
    GO_TO_BALL = 2
    SCORE = 3
    NEUTRAL = 4
    GO_TO_RECEIVE_POSITION = 5
    WAIT_FOR_PASS = 6
    PASS = 7


class DecisionMaker:

    def __init__(self, agent):
        """
        Initialise the DecisionMaker and all FSM state variables.
        Starts in NEUTRAL; the first call to update_current_behavior will
        immediately overwrite this via _check_global_interrupts.
        """
        from mujococodebase.agent import Agent
        self.agent: Agent = agent
        self.is_passer: bool = True
        self.scoring_distance: float = 3.0

        # Beaming and initialization
        self.beam_pose = self._get_beam_pose(random_poses=True)
        self._current_state = State.NEUTRAL # This will be immedietely overwritten
        self.has_initialized = False
        self.grid_scale: int = 10  # each grid cell = 10 cm in world space
        logger.debug(f"[test1] grid world created with scale {self.grid_scale}")

        # Pathfinding
        self.path_targets = {}

        self._last_position: np.ndarray | None = None
        self.planning_threads: dict[str, threading.Thread] = {}
        self.planning_cancel_events: dict[str, threading.Event] = {}
        self.paths = {
            "robot_to_ball": [],
            "scoring": [],
            "passing": [],
            "robot_to_receive": [],
        }
        self.path_ready_events = {
            "robot_to_ball": threading.Event(),
            "scoring": threading.Event(),
            "passing": threading.Event(),
            "robot_to_receive": threading.Event(),
        }
        self.path_steps = {
            "robot_to_ball": 0,
            "scoring": 0,
            "passing": 0,
            "robot_to_receive": 0,
        }
        self.path_follower = PathFollower(agent)
        self._follower_path_id = None  # id(self.paths[key]) last passed to path_follower.set_path
        self.ball_pos_at_last_plan: dict[str, np.ndarray] = {
            "robot_to_ball": np.zeros(2),
            "scoring": np.zeros(2),
            "passing": np.zeros(2),
            "robot_to_receive": np.zeros(2),
        }
        self.time_at_last_plan: dict[str, float] = {
            "robot_to_ball": 0,
            "scoring": 0,
            "passing": 0,
            "robot_to_receive": 0,
        }
        self.replan_cooldown: dict[str, float] = {
            "robot_to_ball": 0.5,
            "scoring": 0.3,
            "passing": 0.3,
            "robot_to_receive": 0.7,
        }
        self.max_no_replan: dict[str, float] = {
            "robot_to_ball": 3,
            "scoring": 1.5,
            "passing": 1.5,
            "robot_to_receive": 4,
        }

        self._ball_kick_speed_streak = 0

        # testing
        self.has_kicked = False
        self._ball_stop_debug_last_ts: float = 0.0
        # Log [metric] joint_torque_max_nm only when the per-step peak hits a new high (reduces spam).
        self._joint_torque_historical_max_nm: float = -1.0

    # --------------------------------------------------
    # Core Loop
    # --------------------------------------------------

    def update_current_behavior(self) -> None:
        if self.agent.world.playmode is PlayModeEnum.GAME_OVER:
            return

        self.is_passer = self._is_passer()
        self._check_global_interrupts()
        self._update_ball_kick_streak()
        self._check_and_replan()
        #print(self._current_state)

        match self._current_state:
            case State.BEAMING:
                self._state_beaming()
            case State.GETTING_UP:
                self._state_getting_up()
            case State.GO_TO_BALL:
                self._state_go_to_ball()
            case State.SCORE:
                self._state_score()
            case State.NEUTRAL:
                self._state_neutral()
            case State.GO_TO_RECEIVE_POSITION:
                self._state_go_to_receive_position()
            case State.WAIT_FOR_PASS:
                self._state_wait_for_pass()
            case State.PASS:
                self._state_pass()
        
        if (self.agent.world.playmode_group not in (PlayModeGroupEnum.ACTIVE_BEAM, PlayModeGroupEnum.PASSIVE_BEAM) and 
            not self.has_initialized):
            self._initialize()

        self._emit_viz_tick()
        self.agent.robot.commit_motor_targets_pd()
        self._log_trial_info()

    # --------------------------------------------------
    # State Transitions
    # --------------------------------------------------

    def _check_global_interrupts(self):
        """Global transitions (higher priority overrides)"""
        # 1. Beaming — always highest priority.
        #    Missing a beam window causes the agent to spawn in the wrong place.
        if self.agent.world.playmode_group in (
            PlayModeGroupEnum.ACTIVE_BEAM,
            PlayModeGroupEnum.PASSIVE_BEAM,
        ):
            self._enter_state(State.BEAMING)
            return

        # 2. GetUp — robot has fallen and must recover before anything else.
        if self.agent.skills_manager.is_ready("GetUp"):
            self._enter_state(State.GETTING_UP)
            return

        # 3. Neutral playmodes — game is paused, no active play behaviour needed.
        if self.agent.world.playmode in (
            PlayModeEnum.BEFORE_KICK_OFF,
            PlayModeEnum.THEIR_GOAL,
            PlayModeEnum.OUR_GOAL,
        ):
            self._enter_state(State.NEUTRAL)
            return

        # 4. Default: enter GO_TO_BALL if not already in an active play state.
        #    GO_TO_BALL will transition to any appropriate active play state automatically.
        if self._current_state not in (
            State.GO_TO_BALL, 
            State.SCORE, 
            State.GO_TO_RECEIVE_POSITION,
            State.WAIT_FOR_PASS,
            State.PASS,
        ) and self.has_initialized:
            self._enter_state(State.GO_TO_BALL)
    
    def _enter_state(self, new_state: State) -> None:
        """Transition to new_state, running any entry actions. Nothing if already in that state."""
        if self._current_state == new_state:
            return

        match new_state:
            case State.BEAMING:
                pass  # No entry actions yet.
            case State.GETTING_UP:
                pass  # No entry actions yet.
            case State.GO_TO_BALL:
                if self.has_initialized:
                    # Force a fresh path from the robot's current pose on GO_TO_BALL entry.
                    self._create_grid_world()
                    self._plan_path("robot_to_ball", self.carry_grid_pos, self.carry_world_pos)
            case State.SCORE:
                # self._replan()
                self._check_and_replan()
                pass
            case State.NEUTRAL:
                pass  # No entry actions yet.
            case State.GO_TO_RECEIVE_POSITION:
                # self._replan()
                self._check_and_replan()
                pass
            case State.WAIT_FOR_PASS:
                pass  # No entry actions yet.
            case State.PASS:
                pass  # No entry actions yet.

        self._current_state = new_state

    # --------------------------------------------------
    # State Functions
    # --------------------------------------------------

    def _state_beaming(self):
        """Commit beam pose each tick"""
        if self.agent.world.playmode_group not in (
            PlayModeGroupEnum.ACTIVE_BEAM,
            PlayModeGroupEnum.PASSIVE_BEAM,
        ):
            self._enter_state(State.NEUTRAL)
            return

        if self.beam_pose is not None:
            pos2d = self.beam_pose[:2]
            rotation = self.beam_pose[2]
        else:
            logger.warning("No valid beam pose generated before beam state reached.")
            return

        self.agent.server.commit_beam(pos2d=pos2d, rotation=rotation)

    def _state_getting_up(self):
        """Execute the GetUp skill. Transition to GO_TO_BALL when finished."""
        finished = self.agent.skills_manager.execute("GetUp")
        if finished:
            self._enter_state(State.GO_TO_BALL)

    def _state_neutral(self):
        """Execute the Neutral skill."""
        self.agent.skills_manager.execute("Neutral")

    def _state_go_to_ball(self):
        """
        Follow the planned path to the position behind the ball, facing the ball-to-goal direction when close.
        Transitions to SCORE if path is completed and inside scoring distance.
        Transitions to PASS if path is completed and outside scoring distance.
        Transitions to GO_TO_RECEIVE_POSITION if not self.is_passer.
        """
        if not self.is_passer:
            self._enter_state(State.GO_TO_RECEIVE_POSITION)
            return
        
        ball_pos = self.agent.world.ball_pos[:2]
        goal_pos = self.agent.world.field.get_their_goal_position()[:2]
        ball_to_goal = goal_pos - ball_pos
        ball_to_goal_norm = np.linalg.norm(ball_to_goal)
        desired_orientation = MathOps.vector_angle(ball_to_goal) if ball_to_goal_norm > 0 else None

        # Only apply orientation when close to the carry position, so the robot doesn't try to rotate while still far away.
        my_pos = self.agent.world.global_position[:2]
        carry_pos = self.carry_world_pos if hasattr(self, "carry_world_pos") else ball_pos
        orientation = desired_orientation if np.linalg.norm(my_pos - carry_pos) <= 2.0 else None

        if self._should_score():
            self._follow_path("robot_to_ball", next_state=State.SCORE, target_orientation=orientation)
        else:
            self._follow_path("robot_to_ball", next_state=State.PASS, target_orientation=orientation)

    def _state_score(self):
        """
        Follow the planned scoring path through the ball toward the goal.
        Falls back to GO_TO_BALL when the path is exhausted.
        Transitions to GO_TO_BALL when the path is complete.
        Transitions to GO_TO_BALL when the robot is out of range of the ball.
        
        """
        EXPECTED_BALL_ERROR = 0.1  # meters

        planned_ball_pos = self.ball_pos_at_last_plan["scoring"]
        current_ball_pos = self.agent.world.ball_pos[:2]

        if np.linalg.norm(current_ball_pos - planned_ball_pos) > EXPECTED_BALL_ERROR:
            self._enter_state(State.GO_TO_BALL)
            return

        my_pos = self.agent.world.global_position[:2]
        ball_pos = self.agent.world.ball_pos[:2]

        SCORING_ABANDON_THRESHOLD = 1.0  # meters
        if np.linalg.norm(my_pos - ball_pos) > SCORING_ABANDON_THRESHOLD:
            self._enter_state(State.GO_TO_BALL)
            return

        goal_pos = self.agent.world.field.get_their_goal_position()[:2]
        ball_to_goal = goal_pos - ball_pos
        ball_to_goal_norm = np.linalg.norm(ball_to_goal)
        desired_orientation = MathOps.vector_angle(ball_to_goal) if ball_to_goal_norm > 0 else None

        self._follow_path("scoring", next_state=State.GO_TO_BALL, target_orientation=desired_orientation)
    
    def _state_pass(self):
        """
        Follow the planned passing path through the ball toward the receive position.
        Falls back to GO_TO_BALL when the path is exhausted.
        Transitions to GO_TO_BALL when the path is complete.
        Transitions to GO_TO_BALL when the robot is out of range of the ball.
        
        """
        EXPECTED_BALL_ERROR = 0.1

        planned_ball_pos = self.ball_pos_at_last_plan["passing"]
        current_ball_pos = self.agent.world.ball_pos[:2]

        if np.linalg.norm(current_ball_pos - planned_ball_pos) > EXPECTED_BALL_ERROR:
            self._enter_state(State.GO_TO_BALL)
            return

        my_pos = self.agent.world.global_position[:2]
        ball_pos = self.agent.world.ball_pos[:2]

        PASSING_ABANDON_THRESHOLD = 1.0  # meters
        if np.linalg.norm(my_pos - ball_pos) > PASSING_ABANDON_THRESHOLD:
            self._enter_state(State.GO_TO_BALL)
            return

        receive_pos = self.receive_world_pos
        ball_to_receive_pos = receive_pos - ball_pos
        ball_to_receive_pos_norm = np.linalg.norm(ball_to_receive_pos)
        desired_orientation = MathOps.vector_angle(ball_to_receive_pos) if ball_to_receive_pos_norm > 0 else None

        self._follow_path("passing", next_state=State.GO_TO_BALL, target_orientation=desired_orientation)

    def _state_go_to_receive_position(self):
        """
        Follow the planned path to the receive position.
        Transitions to NEUTRAL if path is completed.
        Transitions to GO_TO_BALL if is_passer.
        """
        if self.is_passer:
            self._enter_state(State.GO_TO_BALL)
            return
        
        ball_pos = self.agent.world.ball_pos[:2]
        goal_pos = self.agent.world.field.get_their_goal_position()[:2]
        ball_to_goal = goal_pos - ball_pos
        ball_to_goal_norm = np.linalg.norm(ball_to_goal)
        desired_orientation = MathOps.vector_angle(ball_to_goal) if ball_to_goal_norm > 0 else None

        self._follow_path("robot_to_receive", next_state=State.WAIT_FOR_PASS, target_orientation=desired_orientation)
    
    def _state_wait_for_pass(self):
        """
        Hold position at the receive point, turning to face ball
        Transitions to GO_TO_BALL if is_passer.
        Transitions to GO_TO_RECEIVE_POSITION if ball has moved more than threshold
        """

        if self.is_passer:
            self._enter_state(State.GO_TO_BALL)
            return

        my_pos = self.agent.world.global_position[:2]
        ball_pos = self.agent.world.ball_pos[:2]

        RECEIVE_POS_MOVED_THRESHOLD = 1.0 # meters
        if np.linalg.norm(my_pos - self.receive_world_pos) > RECEIVE_POS_MOVED_THRESHOLD:
            self._enter_state(State.GO_TO_RECEIVE_POSITION)
            return

        ball_dir = ball_pos - my_pos
        orientation = MathOps.vector_angle(ball_dir) if np.linalg.norm(ball_dir) > 0 else None

        self.agent.skills_manager.execute(
            "Walk",
            target_2d=my_pos,
            is_target_absolute=True,
            orientation=orientation,
        )

    # --------------------------------------------------
    # Per-Frame Helpers
    # --------------------------------------------------
    
    def _check_and_replan(self):
        """
        If has initialized, replan every replan_cooldown seconds.
        """
        if not self.has_initialized:
            return

        if self._current_state in (State.BEAMING, State.GETTING_UP, State.NEUTRAL):
            return
        
        # Refresh dynamic planning world so replan start/targets use current positions.
        self._create_grid_world()
        
        current_time = time.time()
        my_pos = self.agent.world.global_position[:2]
        ball_pos = self.agent.world.ball_pos[:2]
        currently_kicking = self._ball_kick_speed_streak >= 3

        MUST_REPLAN_THRESHOLD = 6 # proportional to the replan_cooldown

        # robot_to_ball
        if (
            self._current_state == State.GO_TO_BALL
            # make sure it's not too soon to replan
            and current_time - self.time_at_last_plan["robot_to_ball"] > self.replan_cooldown["robot_to_ball"]
            and (
                # check if ball has moved enough since last plan, unless it just got kicked
                not currently_kicking and np.linalg.norm(ball_pos - self.ball_pos_at_last_plan["robot_to_ball"]) >= 0.1
                # check if it's been too long since last plan
                or current_time - self.time_at_last_plan["robot_to_ball"] > self.max_no_replan["robot_to_ball"]
            )
        ):
            self._plan_path("robot_to_ball", self.carry_grid_pos, self.carry_world_pos)

        # scoring
        elif (
            self._current_state == State.SCORE
            # make sure it's not too soon to replan
            and current_time - self.time_at_last_plan["scoring"] > self.replan_cooldown["scoring"]
            and (
                # check if the robot is getting close to the ball and about to kick it
                np.linalg.norm(ball_pos - self.ball_pos_at_last_plan["scoring"]) >= 0.1
                # check if it's been too long since last plan
                or current_time - self.time_at_last_plan["scoring"] > self.max_no_replan["scoring"]
            )
        ):
            self._plan_path("scoring", self.scoring_grid_pos, self.scoring_world_pos)
        
        # passing
        elif (
            self._current_state == State.PASS
            # make sure it's not too soon to replan
            and current_time - self.time_at_last_plan["passing"] > self.replan_cooldown["passing"]
            and (
                # check if the robot is getting close to the ball and about to kick it
                np.linalg.norm(ball_pos - self.ball_pos_at_last_plan["passing"]) >= 0.1
                # check if it's been too long since last plan
                or current_time - self.time_at_last_plan["passing"] > self.max_no_replan["passing"]
            )
        ):
            self._plan_path("passing", self.passing_grid_pos, self.passing_world_pos)

        # robot_to_receive
        elif (
            self._current_state in (State.GO_TO_RECEIVE_POSITION, State.WAIT_FOR_PASS)
            # make sure it's not too soon to replan
            and current_time - self.time_at_last_plan["robot_to_receive"] > self.replan_cooldown["robot_to_receive"]
            and (
                # check if ball has moved enough since last plan, unless it just got kicked
                not currently_kicking and np.linalg.norm(ball_pos - self.ball_pos_at_last_plan["robot_to_receive"]) >= 0.5
                # check if it's been too long since last plan
                or current_time - self.time_at_last_plan["robot_to_receive"] > self.max_no_replan["robot_to_receive"]
            )
        ):
            self._plan_path("robot_to_receive", self.receive_grid_pos, self.receive_world_pos)
    
    def _emit_viz_tick(self) -> None:
        """
        Emit one visualizer update every control tick.

        This keeps robots visible before kickoff (NEUTRAL/BEAMING/GETTING_UP),
        not only while path-following states are active.
        """
        path_key_by_state = {
            State.GO_TO_BALL: "robot_to_ball",
            State.SCORE: "scoring",
            State.GO_TO_RECEIVE_POSITION: "robot_to_receive",
            State.PASS: "passing",
        }

        path_key = path_key_by_state.get(self._current_state)
        plan = self.paths.get(path_key, []) if path_key else []
        current_step = self.path_follower.get_current_waypoint_index(),

        agent_world_pos = self.agent.world.global_position[:2].tolist()
        ball_pos = (
            list(self.agent.world.ball_pos[:2])
            if self.agent.world.is_ball_pos_updated
            else None
        )
        _viz_emit(
            player_num=self.agent.world.number,
            team=self.agent.world.team_name,
            planned_path=plan,
            grid_scale=self.grid_scale,
            current_step=current_step,
            current_pos=agent_world_pos,
            state=self._current_state.name,
            play_mode=self.agent.world.playmode.name,
            target_pos=getattr(self, "_viz_goal_world", None),
            ball_pos=ball_pos,
            is_passer=self.is_passer,
        )
    
    def _log_trial_info(self) -> None:
        if not LOG_METRICS:
            return

        # ── per-timestep metrics ───────────────────────────────────────────
        if any(m in LOG_METRICS for m in ("velocity", "com_z_vel", "com_x_vel")):
            now = time.time()
            pos = self.agent.world.global_position.copy()
            if self._last_position is not None:
                vel_vec = (pos - self._last_position) / SIM_TIMESTEP
                if "velocity" in LOG_METRICS:
                    print(f"[metric] velocity: {np.linalg.norm(vel_vec[:2]):.4f}")
                if "com_z_vel" in LOG_METRICS:
                    print(f"[metric] com_z_vel: {vel_vec[2]:.4f}")
                if "com_x_vel" in LOG_METRICS:
                    print(f"[metric] com_x_vel: {vel_vec[0]:.4f}")
            self._last_position = pos

        if "com_height" in LOG_METRICS:
            print(f"[metric] com_height: {self.agent.world.global_position[2]:.4f}")

        # if "com_z_vel" in LOG_METRICS:
        #     print(f"[metric] com_z_vel: {self.agent.world.global_linvel[2]:.4f}")

        # if "com_x_vel" in LOG_METRICS:
        #     print(f"[metric] com_x_vel: {self.agent.world.global_linvel[0]:.4f}")

        if "latency_ms" in LOG_METRICS:
            _t0 = time.perf_counter()
        # ── end metrics setup ──────────────────────────────────────────────

        # reached_ball — stop trigger for Run E
        if "velocity" in LOG_METRICS:
            print(self.path_steps["robot_to_ball"] >= len(self.paths["robot_to_ball"]) > 0)
            if self.path_steps["robot_to_ball"] >= len(self.paths["robot_to_ball"]) > 0:
                print("[metric] reached_ball")

        # ── post-update metrics ────────────────────────────────────────────
        if "latency_ms" in LOG_METRICS:
            elapsed_ms = (time.perf_counter() - _t0) * 1000
            print(f"[metric] latency_ms: {elapsed_ms:.3f}")

        if LOG_METRICS & _LOG_COMMANDED_PD_TORQUE_KEYS:
            w = self.agent.world
            server_peak = w.mj_leg_actuator_torque_peak_nm
            if server_peak is not None:
                peak_nm = float(server_peak)
                torques = None
            else:
                torques = _commanded_pd_torques_nm(self.agent.robot)
                peak_nm = max((abs(v) for v in torques.values()), default=0.0)
            # Peak torque for tests / plots: prefer MuJoCo leg motors (tauGT), else commanded PD.
            if peak_nm > self._joint_torque_historical_max_nm + 1e-9:
                self._joint_torque_historical_max_nm = peak_nm
                print(f"[metric] joint_torque_max_nm: {peak_nm:.6f}")
            if server_peak is not None:
                if peak_nm > MAX_KICK_TORQUE_NM:
                    print(f'[metric] joint_torques: {json.dumps({"server_leg_peak_nm": peak_nm})}')
            elif torques is not None:
                bad_torques = {k: v for k, v in torques.items() if abs(v) > MAX_KICK_TORQUE_NM}
                if bad_torques:
                    print(f"[metric] joint_torques: {json.dumps(bad_torques)}")

        if "joint_angles" in LOG_METRICS:
            angles = self.agent.robot.motor_positions
            limits = _hinge_joint_limits()
            bad_angles = {}
            for joint, angle in angles.items():
                lo, hi = limits.get(joint, (-180.0, 180.0))
                if not (lo - JOINT_LIMIT_MARGIN_DEG <= angle <= hi + JOINT_LIMIT_MARGIN_DEG):
                    bad_angles[joint] = angle
            if bad_angles:
                print(f"[metric] joint_angles: {json.dumps(bad_angles)}")
        # ── end post-update metrics ────────────────────────────────────────

        if "ball_stopped" in LOG_METRICS:
            w_ball = self.agent.world
            v_ball = w_ball.ball_velocity
            v_xy = float(np.linalg.norm(v_ball[:2]))
            can_measure_ball_stop = w_ball.is_ball_pos_updated or w_ball.ball_velocity_from_gt

            if _ball_stop_debug_enabled():
                now = time.monotonic()
                if now - self._ball_stop_debug_last_ts >= 0.15:
                    self._ball_stop_debug_last_ts = now
                    vn = float(np.linalg.norm(v_ball))
                    print(
                        "[debug ball_stop] "
                        f"state={self._current_state.name} "
                        f"has_kicked={self.has_kicked} "
                        f"metrics_done={getattr(self, '_pass_metrics_logged', False)} "
                        f"ball_seen={w_ball.is_ball_pos_updated} "
                        f"v_norm={vn:.5f} v_xy={v_xy:.5f} "
                        f"ball_xy={w_ball.ball_pos[:2].tolist()} "
                        f"kick_streak={self._ball_kick_speed_streak} "
                        f"would_stop={can_measure_ball_stop and v_xy <= _BALL_STOPPED_SPEED_XY_THRESH}"
                    )

            if not self.has_kicked:
                if self._ball_kick_speed_streak >= 3:
                    self.has_kicked = True
            elif not getattr(self, "_pass_metrics_logged", False):
                if can_measure_ball_stop and v_xy <= _BALL_STOPPED_SPEED_XY_THRESH:
                    self._pass_metrics_logged = True
                    bx = float(self.agent.world.ball_pos[0])
                    by = float(self.agent.world.ball_pos[1])
                    print(f"[metric] ball_stopped: ({bx:.6f}, {by:.6f})", flush=True)
                    if "target_pos" in LOG_METRICS and hasattr(self, "passing_world_pos"):
                        tx = float(self.passing_world_pos[0])
                        ty = float(self.passing_world_pos[1])
                        print(f"[metric] target_pos: ({tx:.6f}, {ty:.6f})", flush=True)
                    print("[test 5,6] ball kicked then stopped", flush=True)


    # --------------------------------------------------
    # Standard Helpers
    # --------------------------------------------------

    def _initialize(self) -> None:
        logger.debug("agent initialization")
        self._create_grid_world()
        for path_key, grid_target, world_target in (
            ("robot_to_ball", self.carry_grid_pos, self.carry_world_pos),
            ("scoring", self.scoring_grid_pos, self.scoring_world_pos),
            ("robot_to_receive", self.receive_grid_pos, self.receive_world_pos),
            ("passing", self.passing_grid_pos, self.passing_world_pos),
        ):
            self._plan_path(path_key, grid_target, world_target)
        self.has_initialized = True

    def _grid_path_to_world_path(self, grid_path: list) -> np.ndarray:
        """Grid cells and degrees → meters and degrees for PathFollower."""
        p = np.asarray(grid_path, dtype=float)
        if p.size == 0:
            return np.zeros((0, 3), dtype=float)
        return np.column_stack((p[:, :2] / float(self.grid_scale), p[:, 2]))

    def _follow_path(self, path_key: str, next_state: State, target_orientation: float | None = None) -> None:
        """
        Follow a planned path one waypoint at a time.

        Waits in a neutral stance until the planning thread signals the path is
        ready. Uses PathFollower on a world-space path (converted from grid).

        Args:
            path_key:    Key into self.paths / self.path_ready_events / self.path_steps.
            next_state:  State to enter once the path is exhausted.
            target_orientation: Desired heading (radians) passed to the Walk skill.
                         None lets the Walk skill choose its own heading.
        """
        # if path planning is incomplete, wait
        if not self.path_ready_events[path_key].is_set():
            target_location = self.path_targets.get(path_key)[:2]
            target_orientation = self.path_targets.get(path_key)[2] if len(self.path_targets.get(path_key)) > 2 else None
            if target_location is not None:
                self.agent.skills_manager.execute(
                    "Walk",
                    target_2d=target_location,
                    is_target_absolute=True,
                    orientation=target_orientation,
                )
            else:
                self.agent.skills_manager.execute("Neutral")
            return

        grid_path = self.paths[path_key]
        if len(grid_path) == 0:
            self._enter_state(next_state)
            return
        
        if self._follower_path_id != id(grid_path):
            self.path_follower.set_path(self._grid_path_to_world_path(grid_path))
            self._follower_path_id = id(grid_path)

        if self.path_follower.is_path_complete():
            self._enter_state(next_state)
            return

        self.path_follower.follow_current_path()

    def _plan_path(self, path_key: str, grid_target: np.ndarray, world_target: np.ndarray):
        """Plan a single path."""
        self.path_targets[path_key] = world_target
        self.path_ready_events[path_key].clear()
        self.path_steps[path_key] = 0
        self.time_at_last_plan[path_key] = time.time()
        self.ball_pos_at_last_plan[path_key] = self.agent.world.ball_pos[:2].copy()

        # Cancel and replace any in-flight planner for this path.
        previous_cancel_event = self.planning_cancel_events.get(path_key)
        if previous_cancel_event is not None:
            previous_cancel_event.set()
        previous_thread = self.planning_threads.get(path_key)
        if previous_thread is not None and previous_thread.is_alive():
            previous_thread.join(timeout=0.05)

        cancel_event = threading.Event()
        self.planning_cancel_events[path_key] = cancel_event

        t = threading.Thread(
            target=planner,
            args=(
                self.grid_world,
                self.agent_grid_pos,
                grid_target,
                path_key,
                self.paths,
                self.path_ready_events,
                cancel_event,
            ),
            daemon=True,
        )
        self.planning_threads[path_key] = t
        t.start()

    def _create_grid_world(self):
        """
        Convert the simulation world to a grid world for planning purposes.
        """
        self.grid_world: GridWorld = GridWorld(
            self.agent.world.field.get_length() * self.grid_scale, 
            self.agent.world.field.get_width() * self.grid_scale
        )
        
        # add obstacle locations (enemies and teammates, excluding self)
        obstacles: list[OtherRobot] = [player for player in self.agent.world.their_team_players if player.last_seen_time is not None]
        obstacles += [player for player in self.agent.world.our_team_players if player.last_seen_time is not None and player is not self.agent]
        for robot in obstacles:
            pos = robot.position
            # logger.debug(f"Obstacle at {pos}")
            self.grid_world.add_obstacle(np.array([round(pos[0] * self.grid_scale), round(pos[1] * self.grid_scale)]), obstacle_radius=ROBOT_RADIUS, inflation_amount=6)

        # convert location of line in front of the goal to grid coordinates
        goal_world_pos = self.agent.world.field.get_their_goal_position()[:2]
        lm = self.agent.world.field.field_landmarks.landmarks
        if "g_lup" in lm and "g_llp" in lm:
            goal_width = float(abs(lm["g_lup"][1] - lm["g_llp"][1]))
        else:
            # Vision may not have seen goal posts yet at kickoff / first _initialize().
            goal_width = float(self.agent.world.field.get_goal_width())
        offsets = np.array(range(
            round((goal_world_pos[1] - goal_width/2) * self.grid_scale), 
            round((goal_world_pos[1] + goal_width/2) * self.grid_scale)
        ))
        x = round(goal_world_pos[0] * self.grid_scale)
        y = np.round(goal_world_pos[1] * self.grid_scale + offsets)
        self.goal_grid_pos = np.column_stack((np.full(len(offsets), x), y))

        # cache the goal world pos for the viz target marker
        self._viz_goal_world = list(goal_world_pos)

        # convert location of ball to grid coordinates
        ball_world_pos = self.agent.world.ball_pos[:2]
        ball_to_goal = goal_world_pos - ball_world_pos
        ball_to_goal_norm = np.linalg.norm(ball_to_goal)
        ball_to_goal_dir = ball_to_goal / ball_to_goal_norm if ball_to_goal_norm > 0 else np.zeros(2)
        self.ball_grid_pos = np.array([
            round(ball_world_pos[0] * self.grid_scale), 
            round(ball_world_pos[1] * self.grid_scale)])

        # convert location of agent to grid coordinates
        agent_world_pos = self.agent.world.global_position[:2] # NOTE: global_position[2] is used for detecting falls and is NOT the 2D orientation
        agent_orientation = self.agent.robot.global_orientation_euler[2]
        self.agent_grid_pos = np.array([
            round(agent_world_pos[0] * self.grid_scale), 
            round(agent_world_pos[1] * self.grid_scale),
            MathOps.normalize_deg(45.0 * round(MathOps.normalize_deg(agent_orientation) / 45.0))
        ])

        # cache the goal world pos for the viz target marker
        self._viz_goal_world = list(goal_world_pos)

        # Carry position: 0.20 m *behind* the ball along the ball-to-goal line.
        self.carry_world_pos = ball_world_pos - ball_to_goal_dir * 0.20
        carry_orientation = MathOps.vector_angle(ball_to_goal) if ball_to_goal_norm > 0 else 0.0
        self.carry_grid_pos = np.array([
            round(self.carry_world_pos[0] * self.grid_scale),
            round(self.carry_world_pos[1] * self.grid_scale),
            MathOps.normalize_deg(45.0 * round(MathOps.normalize_deg(carry_orientation) / 45.0))
        ])

        # Scoring target: 0.20 m *in front of* the ball along the ball-to-goal line.
        self.scoring_world_pos = ball_world_pos + ball_to_goal_dir * 0.20
        scoring_orientation = MathOps.vector_angle(ball_to_goal) if ball_to_goal_norm > 0 else 0.0
        self.scoring_grid_pos = np.array([
            round(self.scoring_world_pos[0] * self.grid_scale),
            round(self.scoring_world_pos[1] * self.grid_scale),
            MathOps.normalize_deg(45.0 * round(MathOps.normalize_deg(scoring_orientation) / 45.0))
        ])

        # Receive position: min of 4 meters from ball to goal along ball-to-goal line,
        # or half way from ball to goal along ball-to-goal line.
        receive_distance = min(4.0, 0.5 * ball_to_goal_norm)
        self.receive_world_pos = ball_world_pos + ball_to_goal_dir * receive_distance
        receive_orientation = MathOps.vector_angle(ball_to_goal) if ball_to_goal_norm > 0 else -180.0
        self.receive_grid_pos = np.array([
            round(self.receive_world_pos[0] * self.grid_scale),
            round(self.receive_world_pos[1] * self.grid_scale),
            MathOps.normalize_deg(45.0 * round(MathOps.normalize_deg(receive_orientation) / 45.0))
        ])

        # Passing target: 0.20 m *in front of* the ball along the ball-to-receive line.
        ball_to_receive = self.receive_world_pos - ball_world_pos
        ball_to_receive_norm = np.linalg.norm(ball_to_receive)
        self.passing_world_pos = ball_world_pos + ball_to_receive * 0.20
        passing_orientation = MathOps.vector_angle(ball_to_receive) if ball_to_receive_norm > 0 else 0.0
        self.passing_grid_pos = np.array([
            round(self.passing_world_pos[0] * self.grid_scale),
            round(self.passing_world_pos[1] * self.grid_scale),
            MathOps.normalize_deg(45.0 * round(MathOps.normalize_deg(passing_orientation) / 45.0))
        ])
    
    def _should_score(self) -> bool:
        """
        Returns True if the ball it withint a distance from the goal.
        """
        ball_pos = self.agent.world.ball_pos[:2]
        goal_pos = self.agent.world.field.get_their_goal_position()[:2]
        ball_to_goal = ball_pos - goal_pos
        return np.linalg.norm(ball_to_goal) < self.scoring_distance
    
    def _is_passer(self) -> bool:
        """
        Returns True if this agent is closer to a considered pos than all teammates.
        Considered position is carry_world_pos if it exists. Otherwise, is ball_pos.

        Visibility is determined by last_seen_time being set on the OtherRobot object.
        If no teammates are visible, returns True, since it is assumed it is the only player.

        Returns:
            bool: True if this agent is the closest to the considered pos. False otherwise.
        """
        ball_pos = self.agent.world.ball_pos[:2]
        considered_pos = self.carry_world_pos if hasattr(self, "carry_world_pos") else ball_pos
        my_pos = self.agent.world.global_position[:2]
        my_dist = np.linalg.norm(my_pos - considered_pos)

        teammates = [
            player for player in self.agent.world.our_team_players
            if player.last_seen_time is not None # only populated slots
            and np.linalg.norm(player.position[:2] - my_pos) > 0.1 # exclude self
        ]

        if not teammates:
            if "solo" not in LOG_METRICS:
                logger.debug(f"No teammate positions available. Holding current role: {'passer' if self.is_passer else 'receiver'}")
            return bool(self.is_passer)  # hold current role if no teammate data yet

        closest_teammate_dist = min(np.linalg.norm(p.position[:2] - considered_pos) for p in teammates)
        
        EPS = 1e-3
        if my_dist < closest_teammate_dist - EPS:
            return True
        elif my_dist > closest_teammate_dist + EPS:
            return False
        else:
            # deterministic tie-breaker
            return self.agent.world.number == 1

    def _update_ball_kick_streak(self) -> None:
        """Advance kick-detection streak from vision-only velocity (not simulator ballGT).

        Ground-truth ``ball_velocity`` can be >0.5 m/s while the ball is settling at spawn;
        vision finite-difference velocity stays near zero until the ball actually moves in view.

        Only ``PLAY_ON`` counts so kickoff / set-piece ball motion does not arm ``has_kicked``
        before the pass. Updated once per frame.
        """
        w = self.agent.world
        if w.playmode != PlayModeEnum.PLAY_ON:
            self._ball_kick_speed_streak = 0
            return
        if not w.is_ball_pos_updated:
            self._ball_kick_speed_streak = 0
            return
        if np.linalg.norm(w.ball_velocity_vision) > 0.5:
            self._ball_kick_speed_streak += 1
        else:
            self._ball_kick_speed_streak = 0

    def _get_beam_pose(self, random_poses: bool):
        """
        Returns the beam pose (x, y, rotation_deg) for this agent.

        Priority order:
            1. Custom pose from PLAYER_SPAWN_X/Y/ROT environment variables.
            2. Random pose within predefined zones (if random_poses=True).
            3. Fixed default pose based on agent number.

        Falls back to origin (0, 0, 0) if an unsupported configuration is
        encountered, logging a warning in each case.

        Args:
            random_poses: If True, randomise spawn position within each agent's designated zone.

        Returns:
            A (x, y, rotation_deg) tuple representing the beam pose.
        """
        # 1. Custom pose takes priority over everything else.
        x = os.getenv("PLAYER_SPAWN_X")
        y = os.getenv("PLAYER_SPAWN_Y")
        r = os.getenv("PLAYER_SPAWN_ROT")
        if x is not None and y is not None and r is not None:
            try:
                return (float(x), float(y), float(r))
            except ValueError:
                logger.warning("Invalid default pose variables; falling back to origin.")
                return (0.0, 0.0, 0.0)
    
        # 2. Random pose within each agent's designated spawn zone.
        if random_poses:
            if self.agent.world.number == 1:
                return (random.uniform(1, 4), random.uniform(0.7, 4), 0)
            elif self.agent.world.number == 2:
                return (random.uniform(1, 4), random.uniform(-0.7, -4), 0)
            else:
                logger.warning("Agent has no random spawn zone defined; falling back to origin.")
                return (0.0, 0.0, 0.0)
        
        # 3. Fixed default poses.
        if self.agent.world.number == 1:
            return (7.0, 0.0, 0)
        elif self.agent.world.number == 2:
            return (2.0, -1.5, 0)
        elif self.agent.world.number == 3:
            return (2.0, 1.5, 0)
        else:
            logger.warning("Agent has no default pose defined; falling back to origin.")
            return (0.0, 0.0, 0.0)


# --------------------------------------------------
# Testing Helpers
# --------------------------------------------------

def _commanded_pd_torques_nm(robot) -> dict[str, float]:
    """
    Commanded actuator torque (Nm) from the PD rule used by the simulator:

        tau = kp * (q_cmd - q) + kd * (dq_cmd - dq) + tau_ff

    Perception and motor effector messages use joint angle/velocity in degrees;
    the server converts q, dq to radians before applying gains.

    This matches the client's motor-command path, not a direct read of mjData.qfrc_*;
    use for test 5B-style caps on commanded torque.
    """
    torques: dict[str, float] = {}
    for name in robot.ROBOT_MOTORS:
        t = robot.motor_targets[name]
        q_cmd = math.radians(t["target_position"])
        dq_cmd = 0.0
        tau_ff = 0.0
        q_deg = robot.motor_positions.get(name, 0.0)
        dq_deg = robot.motor_speeds.get(name, 0.0)
        q = math.radians(q_deg)
        dq = math.radians(dq_deg)
        kp, kd = t["kp"], t["kd"]
        torques[name] = kp * (q_cmd - q) + kd * (dq_cmd - dq) + tau_ff
    return torques

def _load_joint_limits_from_robot_xml() -> dict[str, tuple[float, float]]:
    """Hinge joint (deg) limits from the ant robot model — same source as verification/conftest.py."""
    base = Path(__file__).resolve().parent
    candidates = [
        base.parent.parent / "rcssservermj" / "src" / "rcsssmj" / "resources" / "robots" / "ant" / "robot.xml",
        base.parent.parent / "RCSSServerMJ" / "src" / "rcsssmj" / "resources" / "robots" / "ant" / "robot.xml",
    ]
    robot_xml_path = next((p for p in candidates if p.is_file()), None)
    if robot_xml_path is None:
        raise FileNotFoundError(
            "Could not locate ant robot.xml for joint limit checks. Tried: "
            + ", ".join(str(p) for p in candidates)
        )

    root = ET.parse(robot_xml_path).getroot()
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall(".//joint[@type='hinge'][@range]"):
        name = joint.attrib.get("name")
        range_str = joint.attrib.get("range", "")
        if not name:
            continue
        parts = range_str.split()
        if len(parts) != 2:
            continue
        lo, hi = float(parts[0]), float(parts[1])
        limits[name] = (lo, hi)

    if not limits:
        raise ValueError(f"No hinge joint limits found in {robot_xml_path}")
    return limits

def _hinge_joint_limits() -> dict[str, tuple[float, float]]:
    global _joint_limits_cache
    if _joint_limits_cache is None:
        _joint_limits_cache = _load_joint_limits_from_robot_xml()
    return _joint_limits_cache