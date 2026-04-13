import logging
import os
import threading
import random
import time
from enum import Enum

import numpy as np
from mujococodebase.utils.math_ops import MathOps
from mujococodebase.world.play_mode import PlayModeEnum, PlayModeGroupEnum
from mujococodebase.world.grid_world import GridWorld
from mujococodebase.planning.planning import ana_theta_star as planner
from mujococodebase.planning.path_follower import PathFollower
from mujococodebase.world.other_robot import OtherRobot
from mujococodebase.planning.path_viz_emitter import emit as _viz_emit


logger = logging.getLogger(__file__)


class State(Enum):
    BEAMING = 0
    GETTING_UP = 1
    GO_TO_BALL = 2
    DRIBBLE = 3
    NEUTRAL = 4
    GO_TO_RECEIVE_POSITION = 5
    WAIT_FOR_PASS = 6


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

        # Beaming and initialization
        self.beam_pose = self._get_beam_pose(random_poses=True)
        self._current_state = State.NEUTRAL # This will be immedietely overwritten
        self.has_initialized = False
        self.grid_scale: int = 10  # each grid cell = 10 cm in world space

        # Pathfinding
        self.path_targets = {}
        self.planning_threads: dict[str, threading.Thread] = {}
        self.planning_cancel_events: dict[str, threading.Event] = {}
        self.paths = {
            "robot_to_ball": [],
            "dribble": [],
            "robot_to_receive": [],
        }
        self.path_ready_events = {
            "robot_to_ball": threading.Event(),
            "dribble": threading.Event(),
            "robot_to_receive": threading.Event(),
        }
        self.path_steps = {
            "robot_to_ball": 0,
            "dribble": 0,
            "robot_to_receive": 0,
        }
        self.path_follower = PathFollower(agent)
        self._follower_path_id = None  # id(self.paths[key]) last passed to path_follower.set_path
        self.ball_pos_at_last_plan: dict[str, np.ndarray] = {
            "robot_to_ball": np.zeros(2),
            "dribble": np.zeros(2),
            "robot_to_receive": np.zeros(2),
        }
        self.time_at_last_plan: dict[str, float] = {
            "robot_to_ball": 0,
            "dribble": 0,
            "robot_to_receive": 0,
        }
        self.replan_cooldown: dict[str, float] = {
            "robot_to_ball": 0.5,
            "dribble": 0.3,
            "robot_to_receive": 0.7,
        }

    # --------------------------------------------------
    # Core Loop
    # --------------------------------------------------

    def update_current_behavior(self) -> None:
        if self.agent.world.playmode is PlayModeEnum.GAME_OVER:
            return

        self.is_passer = self._is_closest_to_ball()
        self._check_global_interrupts()
        self._check_and_replan()
        #print(self._current_state)

        match self._current_state:
            case State.BEAMING:
                self._state_beaming()
            case State.GETTING_UP:
                self._state_getting_up()
            case State.GO_TO_BALL:
                self._state_go_to_ball()
            case State.DRIBBLE:
                self._state_dribble()
            case State.NEUTRAL:
                self._state_neutral()
            case State.GO_TO_RECEIVE_POSITION:
                self._state_go_to_receive_position()
            case State.WAIT_FOR_PASS:
                self._state_wait_for_pass()
        
        if (self.agent.world.playmode_group not in (PlayModeGroupEnum.ACTIVE_BEAM, PlayModeGroupEnum.PASSIVE_BEAM) and 
            not self.has_initialized):
            self._initialize()

        self.agent.robot.commit_motor_targets_pd()

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
            State.DRIBBLE, 
            State.GO_TO_RECEIVE_POSITION,
            State.WAIT_FOR_PASS,
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
                pass  # No entry actions yet.
            case State.DRIBBLE:
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
        Transitions to DRIBBLE if path is completed.
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

        self._follow_path("robot_to_ball", next_state=State.DRIBBLE, target_orientation=orientation)

    def _state_dribble(self):
        """
        Follow the planned dribble path through the ball toward the goal.
        Falls back to GO_TO_BALL when the path is exhausted.
        Transitions to GO_TO_BALL when the path is complete.
        Transitions to GO_TO_BALL when the robot is out of range of the ball.
        
        """
        my_pos = self.agent.world.global_position[:2]
        ball_pos = self.agent.world.ball_pos[:2]

        DRIBBLE_ABANDON_THRESHOLD = 1.0  # meters
        if np.linalg.norm(my_pos - ball_pos) > DRIBBLE_ABANDON_THRESHOLD:
            self._enter_state(State.GO_TO_BALL)
            return

        goal_pos = self.agent.world.field.get_their_goal_position()[:2]
        ball_to_goal = goal_pos - ball_pos
        ball_to_goal_norm = np.linalg.norm(ball_to_goal)
        desired_orientation = MathOps.vector_angle(ball_to_goal) if ball_to_goal_norm > 0 else None

        self._follow_path("dribble", next_state=State.GO_TO_BALL, target_orientation=desired_orientation)

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
        
        current_time = time.time()
        my_pos = self.agent.world.global_position[:2]
        ball_pos = self.agent.world.ball_pos[:2]
        currently_kicking = self._is_ball_in_kicking_motion(ball_pos)

        # robot_to_ball
        if (
            self._current_state == State.GO_TO_BALL
            # time guard
            and current_time - self.time_at_last_plan["robot_to_ball"] > self.replan_cooldown["robot_to_ball"]
            # check if ball has moved enough since last plan, unless it just got kicked
            and not currently_kicking and np.linalg.norm(ball_pos - self.ball_pos_at_last_plan["robot_to_ball"]) >= 0.2
        ):
            self._plan_path("robot_to_ball", self.carry_grid_pos, self.carry_world_pos)
            self.time_at_last_plan["robot_to_ball"] = current_time
            self.ball_pos_at_last_plan["robot_to_ball"] = ball_pos.copy()

        # dribble
        elif (
            self._current_state == State.DRIBBLE
            # time guard
            and current_time - self.time_at_last_plan["dribble"] > self.replan_cooldown["dribble"]
            # check if the robot is getting close to the ball and about to kick it
            and np.linalg.norm(my_pos - self.dribble_world_pos) <= 0.2
        ):
            self._plan_path("dribble", self.dribble_grid_pos, self.dribble_world_pos)
            self.time_at_last_plan["dribble"] = current_time
            self.ball_pos_at_last_plan["dribble"] = ball_pos.copy()

        # robot_to_receive
        elif (
            self._current_state in (State.GO_TO_RECEIVE_POSITION, State.WAIT_FOR_PASS)
            # time guard
            and current_time - self.time_at_last_plan["robot_to_receive"] > self.replan_cooldown["robot_to_receive"]
            # check if ball has moved enough since last plan, unless it just got kicked
            and not currently_kicking and np.linalg.norm(ball_pos - self.ball_pos_at_last_plan["robot_to_receive"]) >= 0.5
        ):
            self._plan_path("robot_to_receive", self.receive_grid_pos, self.receive_world_pos)
            self.time_at_last_plan["robot_to_receive"] = current_time
            self.ball_pos_at_last_plan["robot_to_receive"] = ball_pos.copy()

    # --------------------------------------------------
    # Standard Helpers
    # --------------------------------------------------

    def _initialize(self) -> None:
        logger.debug("agent initialization")
        self._create_grid_world()
        for path_key, grid_target, world_target in (
            ("robot_to_ball", self.carry_grid_pos, self.carry_world_pos),
            ("dribble", self.dribble_grid_pos, self.dribble_world_pos),
            ("robot_to_receive", self.receive_grid_pos, self.receive_world_pos),
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

        # ── PATH VIZ ──────────────────────────────────────────────────────────
        agent_world_pos = self.agent.world.global_position[:2].tolist()
        _viz_emit(
            player_num=self.agent.world.number,
            team=self.agent.world.team_name,
            planned_path=self.paths[path_key],
            grid_scale=self.grid_scale,
            current_step=self.path_follower.get_current_waypoint_index(),
            current_pos=agent_world_pos,
            target_pos=getattr(self, "_viz_goal_world", None),
        )
        # ── END VIZ ───────────────────────────────────────────────────────────

        self.path_follower.follow_current_path()

    def _plan_path(self, path_key: str, grid_target: np.ndarray, world_target: np.ndarray):
        """Plan a single path."""
        self.path_targets[path_key] = world_target
        self.path_ready_events[path_key].clear()
        self.path_steps[path_key] = 0

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
        logger.debug(f"[test1] grid world created with scale {self.grid_scale}")
        
        # add obstacle locations (enemies and teammates, excluding self)
        obstacles: list[OtherRobot] = [player for player in self.agent.world.their_team_players if player.last_seen_time is not None]
        obstacles += [player for player in self.agent.world.our_team_players if player.last_seen_time is not None and player is not self.agent]
        for robot in obstacles:
            pos = robot.position
            logger.debug(f"Obstacle at {pos}")
            self.grid_world.add_obstacle(np.array([round(pos[0] * self.grid_scale), round(pos[1] * self.grid_scale)]), inflation_amount=6)

        # convert location of line in front of the goal to grid coordinates
        goal_world_pos = self.agent.world.field.get_their_goal_position()[:2]
        goal_width = abs(self.agent.world.field.field_landmarks.landmarks["g_lup"][1] 
                         - self.agent.world.field.field_landmarks.landmarks["g_llp"][1])
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

        # Carry position: 0.30 m *behind* the ball along the ball-to-goal line.
        self.carry_world_pos = ball_world_pos - ball_to_goal_dir * 0.30
        carry_orientation = MathOps.vector_angle(ball_to_goal) if ball_to_goal_norm > 0 else 0.0
        self.carry_grid_pos = np.array([
            round(self.carry_world_pos[0] * self.grid_scale),
            round(self.carry_world_pos[1] * self.grid_scale),
            MathOps.normalize_deg(45.0 * round(MathOps.normalize_deg(carry_orientation) / 45.0))
        ])

        # Dribble target: 0.30 m *in front of* the ball along the ball-to-goal line.
        self.dribble_world_pos = ball_world_pos + ball_to_goal_dir * 0.30
        dribble_orientation = MathOps.vector_angle(ball_to_goal) if ball_to_goal_norm > 0 else 0.0
        self.dribble_grid_pos = np.array([
            round(self.dribble_world_pos[0] * self.grid_scale),
            round(self.dribble_world_pos[1] * self.grid_scale),
            MathOps.normalize_deg(45.0 * round(MathOps.normalize_deg(dribble_orientation) / 45.0))
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
    
    def _is_closest_to_ball(self) -> bool:
        """
        Returns True if this agent is closer to the ball than all teammates.

        Visibility is determined by last_seen_time being set on the OtherRobot object.
        If no teammates are visible, returns True, since it is assumed it is the only player.

        Returns:
            bool: True if this agent is the closest to the ball, False otherwise.
        """
        ball_pos = self.agent.world.ball_pos[:2]
        my_pos = self.agent.world.global_position[:2]
        my_dist = np.linalg.norm(my_pos - ball_pos)

        teammates = [
            player for player in self.agent.world.our_team_players
            if player.last_seen_time is not None # only populated slots
            and np.linalg.norm(player.position[:2] - my_pos) > 0.1 # exclude self
        ]

        if not teammates:
            logger.debug(f"No teammate positions available. Holding current role: {'passer' if self.is_passer else 'receiver'}")
            return self.is_passer # hold current role if no teammate data yet

        closest_teammate_dist = min(np.linalg.norm(p.position[:2] - ball_pos) for p in teammates)
        return my_dist <= closest_teammate_dist

    def _is_ball_in_kicking_motion(self, ball_velocity: np.ndarray) -> bool:
        """
        Returns True if the ball was kicked and is now in motion.
        """
        return np.linalg.norm(ball_velocity) > 0.5 # 0.5 m/s threshold

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
