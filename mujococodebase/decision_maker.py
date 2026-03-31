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
from mujococodebase.world.planning import ana_theta_star as planner


logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__file__)


class State(Enum):
    BEAMING = 0
    GETTING_UP = 1
    GO_TO_BALL = 2
    DRIBBLE = 3
    NEUTRAL = 4


class DecisionMaker:

    def __init__(self, agent):
        """
        Initialise the DecisionMaker and all FSM state variables.
        Starts in NEUTRAL; the first call to update_current_behavior will
        immediately overwrite this via _check_global_interrupts.
        """

        from mujococodebase.agent import Agent
        self.agent: Agent = agent

        # Beaming and initialization
        self.beam_pose = self._get_beam_pose(random_poses=True)
        self._current_state = State.NEUTRAL # This will be immedietely overwritten
        self.has_initialized = False
        self.grid_scale: int = 10  # each grid cell = 10 cm in world space

        # Pathfinding re-plan characteristics
        self.ball_pos_at_last_plan: np.ndarray | None = None # Ball pos during last pathfinding planning
        self.replan_pos_threshold: float = 0.1 # Minimum distance in meters for replan to occur
        self.time_at_last_plan: float = 0.0
        self.replan_cooldown: float = 3.0  # seconds

        # Pathfinding
        self.planning_threads = []
        self.paths = {
            "robot_to_ball": [],
            "dribble": [],
        }
        self.path_ready_events = {
            "robot_to_ball": threading.Event(),
            "dribble": threading.Event(),
        }
        self.path_steps = {
            "robot_to_ball": 0,
            "dribble": 0,
        }

    # --------------------------------------------------
    # Core Loop
    # --------------------------------------------------

    def update_current_behavior(self) -> None:
        if self.agent.world.playmode is PlayModeEnum.GAME_OVER:
            return

        self._check_global_interrupts()
        self._check_for_replan()

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
        #    GO_TO_BALL and DRIBBLE transition between each other internally.
        if self._current_state not in (State.GO_TO_BALL, State.DRIBBLE) and self.has_initialized:
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
                self._replan()
            case State.NEUTRAL:
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
        Follow the planned path to the carry position behind the ball,
        facing the ball-to-goal direction when close. Hands off to DRIBBLE.
        """
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
        The dribble path points ahead of the ball so the agent pushes it toward the goal.
        """
        ball_pos = self.agent.world.ball_pos[:2]
        goal_pos = self.agent.world.field.get_their_goal_position()[:2]
        ball_to_goal = goal_pos - ball_pos
        ball_to_goal_norm = np.linalg.norm(ball_to_goal)
        desired_orientation = MathOps.vector_angle(ball_to_goal) if ball_to_goal_norm > 0 else None

        self._follow_path("dribble", next_state=State.GO_TO_BALL, target_orientation=desired_orientation)

    # --------------------------------------------------
    # Per-Frame Helpers
    # --------------------------------------------------
    
    def _check_for_replan(self):
        """
        Trigger a full replan if the ball has moved beyond replan_pos_threshold
        since the last plan and the cooldown has elapsed. Skipped until initialized.
        """
        if not self.has_initialized:
            return
        if self.ball_pos_at_last_plan is None:
            return
        if time.time() - self.time_at_last_plan < self.replan_cooldown:
            return

        current_ball_pos = self.agent.world.ball_pos[:2]
        if np.linalg.norm(current_ball_pos - self.ball_pos_at_last_plan) > self.replan_pos_threshold:
            self._replan()

    # --------------------------------------------------
    # Standard Helpers
    # --------------------------------------------------

    def _initialize(self) -> None:
        logger.debug("agent initialization")

        self._create_grid_world()
        self._plan_paths()
        self.ball_pos_at_last_plan = self.agent.world.ball_pos[:2].copy()
        self.has_initialized = True

    def _follow_path(self, path_key: str, next_state: State, target_orientation: float | None = None) -> None:
        """
        Follow a planned path one waypoint at a time.

        Waits in a neutral stance until the planning thread signals the path is
        ready. Advances through waypoints using a proximity threshold, then
        transitions to next_state when the path is fully walked.

        Args:
            path_key:    Key into self.paths / self.path_ready_events / self.path_steps.
            next_state:  State to enter once the path is exhausted.
            target_orientation: Desired heading (radians) passed to the Walk skill.
                         None lets the Walk skill choose its own heading.
        """

        # if path planning is incomplete, wait
        if not self.path_ready_events[path_key].is_set():
            last = getattr(self, "_last_waypoints", {}).get(path_key)
            if last is not None:
                target_location = np.array(last, dtype=float) / self.grid_scale
                self.agent.skills_manager.execute(
                    "Walk",
                    target_2d=target_location,
                    is_target_absolute=True,
                    orientation=target_orientation,
                )
            else:
                self.agent.skills_manager.execute("Neutral")
            return

        # if path has been followed, wait
        if self.path_steps[path_key] >= len(self.paths[path_key]):
            self._enter_state(next_state)
            return

        # Walk to the current waypoint.
        target_location = (np.array(self.paths[path_key][self.path_steps[path_key]], dtype=float) / self.grid_scale)
        self.agent.skills_manager.execute(
            "Walk",
            target_2d=target_location,
            is_target_absolute=True,
            orientation=target_orientation,
        )

        # check if we've arrived and should head to the next waypoint
        agent_location = self.agent.world.global_position[:2]
        agent_to_target = target_location - agent_location
        PATH_COMPLETE_THRESHOLD = 0.1
        if np.linalg.norm(target_location - agent_location) <= PATH_COMPLETE_THRESHOLD:
            self.path_steps[path_key] += 1

    def _replan(self):
        """Discard all current paths and replan."""
        self._last_waypoints = {
            key: self.paths[key][self.path_steps[key]] if self.paths[key] and self.path_steps[key] < len(self.paths[key]) else None
            for key in self.paths
        }

        for key in list(self.paths.keys()):
            self.paths[key] = []
            self.path_ready_events[key].clear()
            self.path_steps[key] = 0
        self.planning_threads = [t for t in self.planning_threads if t.is_alive()]
        self._create_grid_world()
        self._plan_paths()
        self.ball_pos_at_last_plan = self.agent.world.ball_pos[:2]
        self.time_at_last_plan = time.time()

    def _plan_paths(self):
        """
        Plan all necessary paths.
        """
        for path_key, target in (
            ("robot_to_ball", self.carry_grid_pos),
            ("dribble", self.dribble_grid_pos),
        ):
            t = threading.Thread(
                target=planner,
                args=(
                    self.grid_world,
                    self.agent_grid_pos,
                    target,
                    path_key,
                    self.paths,
                    self.path_ready_events,
                ),
            )
            self.planning_threads.append(t)
            t.start()

    def _create_grid_world(self):
        """
        Convert the simulation world to a grid world for planning purposes.
        """
        self.grid_world: GridWorld = GridWorld(
            self.agent.world.field.get_length() * self.grid_scale, 
            self.agent.world.field.get_width() * self.grid_scale
        )

        # add obstacle locations
        obstacles: list[OtherRobot] = [player for player in self.agent.world.their_team_players if player.last_seen_time is not None]
        obstacles: np.ndarray[float] = [robot.position for robot in obstacles]
        for pos in obstacles:
            logger.debug(f"Obstacle at {pos}")
            self.grid_world.add_obstacle(np.array([round(pos[0] * self.grid_scale), round(pos[1] * self.grid_scale)]))

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
        self.agent_grid_pos = np.array([
            round(agent_world_pos[0] * self.grid_scale), 
            round(agent_world_pos[1] * self.grid_scale)])
        # TODO: add orientation information

        # Carry position: 0.30 m *behind* the ball along the ball-to-goal line.
        self.carry_world_pos = ball_world_pos - ball_to_goal_dir * 0.30
        self.carry_grid_pos = np.array([
            round(self.carry_world_pos[0] * self.grid_scale),
            round(self.carry_world_pos[1] * self.grid_scale),
        ])

        # Dribble target: 0.30 m *in front of* the ball along the ball-to-goal line.
        dribble_world_pos = ball_world_pos + ball_to_goal_dir * 0.30
        self.dribble_grid_pos = np.array([
            round(dribble_world_pos[0] * self.grid_scale),
            round(dribble_world_pos[1] * self.grid_scale),
        ])

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
                return (random.uniform(1, 4), random.uniform(0.5, 4), 0)
            elif self.agent.world.number == 2:
                return (random.uniform(1, 4), random.uniform(-0.5, -4), 0)
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