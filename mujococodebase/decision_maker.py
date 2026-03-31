from dataclasses import Field
import logging
import os
import threading
import random
from enum import Enum
from typing import Mapping

import numpy as np
from mujococodebase.utils.math_ops import MathOps
from mujococodebase.world.field import FIFAField, HLAdultField
from mujococodebase.world.play_mode import PlayModeEnum, PlayModeGroupEnum
from mujococodebase.world.grid_world import GridWorld
from mujococodebase.world.planning import ana_theta_star as planner
from mujococodebase.world.other_robot import OtherRobot


logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__file__)


class State(Enum):
    BEAMING = 0
    GETTING_UP = 1
    GO_TO_BALL = 2
    DRIBBLE = 3
    NEUTRAL = 4
    KICKOFF = 5


class DecisionMaker:

    def __init__(self, agent):
        from mujococodebase.agent import Agent
        self.agent: Agent = agent
        self.is_getting_up: bool = False

        self.beam_pose = self._get_beam_pose(True)
        self._current_state = State.NEUTRAL

        self.kicked_off = False

        # configuration for planning
        self.planning_threads = []
        self.paths = {
            "robot_to_ball": [],
            "robot_to_goal": [], 
            "ball_to_goal": []
        }
        self.path_ready_events = {
            "robot_to_ball": threading.Event(),
            "robot_to_goal": threading.Event(),
            "ball_to_goal": threading.Event()
        }
        self.path_steps = {
            "robot_to_ball": 0,
            "robot_to_goal": 0,
            "ball_to_goal": 0
        }

    def _kickoff(self):
        """Initialization tasks that require information from the server that isn't present when __init__() runs."""
        self.kicked_off = True
        self._create_grid_world()
        self._plan_paths()

    # --------------------------------------------------
    # Core Loop
    # --------------------------------------------------

    def update_current_behavior(self) -> None:
        if self.agent.world.playmode is PlayModeEnum.GAME_OVER:
            return

        self._check_global_interrupts()

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
            case State.KICKOFF:
                self._state_kickoff()

        self.agent.robot.commit_motor_targets_pd()

    # --------------------------------------------------
    # State Transitions
    # --------------------------------------------------

    def _check_global_interrupts(self):
        """Global transitions (higher priority overrides)"""

        if self.agent.world.playmode_group in (
            PlayModeGroupEnum.ACTIVE_BEAM,
            PlayModeGroupEnum.PASSIVE_BEAM,
        ):
            self._enter_state(State.BEAMING)
            return

        if self.agent.skills_manager.is_ready("GetUp"):
            self._enter_state(State.GETTING_UP)
            return

        if self.agent.world.playmode in (
            PlayModeEnum.BEFORE_KICK_OFF,
            PlayModeEnum.THEIR_GOAL,
            PlayModeEnum.OUR_GOAL,
        ):
            self._enter_state(State.NEUTRAL)
            return
        
        if self.agent.world.playmode == PlayModeEnum.OUR_KICK_OFF and not self.kicked_off:
            self._enter_state(State.KICKOFF)
            return
        
        if self.agent.world.playmode == PlayModeEnum.OUR_KICK_OFF and self.kicked_off:
            self._enter_state(State.GO_TO_BALL)
            return

        # All other playmodes (including PLAY_ON) → carry ball.
        # Mirrors the original's final `else: carry_ball()` catch-all.
        # GO_TO_BALL / DRIBBLE transition between each other internally.
        if self._current_state not in (State.GO_TO_BALL, State.DRIBBLE):
            self._enter_state(State.GO_TO_BALL)

    # --------------------------------------------------
    # State Functions
    # --------------------------------------------------

    def _state_beaming(self):
        # Exit when the beam phase ends
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

        self.agent.server.commit_beam(pos2d=pos2d, rotation=rotation)

    def _state_kickoff(self):
        # note: this state only runs once and immediately switches to play states
        self._kickoff()
        self.agent.skills_manager.execute("Neutral")

    def _state_getting_up(self):
        finished = self.agent.skills_manager.execute("GetUp")
        if finished:
            self._enter_state(State.GO_TO_BALL)

    def _state_neutral(self):
        self.agent.skills_manager.execute("Neutral")

    def _state_go_to_ball(self):
        # Exit: DRIBBLE

        # if path planning is incomplete, wait
        if not self.path_ready_events["robot_to_ball"].is_set():
            self.agent.skills_manager.execute("Neutral")
            return
        
        # if path has been followed, wait
        if self.path_steps["robot_to_ball"] >= len(self.paths["robot_to_ball"]):
            self.agent.skills_manager.execute("Neutral")
            self._enter_state(State.DRIBBLE)
            # TODO: to safely dribble, we may need to plan to right before the ball, not directly to the ball
            return
        
        # follow robot-to-ball path
        target_location = np.array(self.paths["robot_to_ball"][self.path_steps["robot_to_ball"]], dtype=float) / self.grid_scale
        # TODO: add orientation
        target_orientation = None
        self.agent.skills_manager.execute(
            "Walk",
            target_2d=target_location,
            is_target_absolute=True,
            orientation=target_orientation
        )
        # logger.debug(f"Walking to {target_location}")

        # check if we've arrived and should head to the next waypoint
        agent_location = self.agent.world.global_position[:2]
        agent_to_target = target_location - agent_location
        PATH_COMPLETE_THRESHOLD = 0.1
        if np.linalg.norm(agent_to_target) <= PATH_COMPLETE_THRESHOLD:
            self.path_steps["robot_to_ball"] += 1

    def _state_dribble(self):
        # Exit: GO_TO_BALL if alignment lost

        aligned, behind_ball, _, goal_pos, desired_orientation = self._compute_ball_geometry()

        if goal_pos is None:  # degenerate case: ball on goal line
            return

        if not aligned or not behind_ball:
            self._enter_state(State.GO_TO_BALL)
            return

        self.agent.skills_manager.execute(
            "Walk",
            target_2d=goal_pos,
            is_target_absolute=True,
            orientation=desired_orientation
        )

    # --------------------------------------------------
    # State Entry
    # --------------------------------------------------

    def _enter_state(self, new_state: State):
        if self._current_state == new_state:
            return

        match new_state:
            case State.BEAMING:
                pass  # no entry actions yet
            case State.GETTING_UP:
                pass  # no entry actions yet
            case State.GO_TO_BALL:
                pass  # no entry actions yet
            case State.DRIBBLE:
                pass  # no entry actions yet
            case State.NEUTRAL:
                pass  # no entry actions yet
            case State.KICKOFF:
                pass

        self._current_state = new_state

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------

    def _plan_paths(self):
        """
        Plan all necessary paths.
        """
        t = threading.Thread(
            target=planner, 
            args=(self.grid_world, self.agent_grid_pos, self.ball_grid_pos, "robot_to_ball", self.paths, self.path_ready_events)
        )
        self.planning_threads.append(t)
        t.start()

        # t = threading.Thread(
        #     target=planner, 
        #     args=(self.grid_world, self.agent_grid_pos, self.goal_grid_pos, "robot_to_goal", self.paths, self.path_ready_events)
        # )
        # self.planning_threads.append(t)
        # t.start()

        # t = threading.Thread(
        #     target=planner, 
        #     args=(self.grid_world, self.ball_grid_pos, self.goal_grid_pos, "ball_to_goal", self.paths, self.path_ready_events)
        # )
        # self.planning_threads.append(t)
        # t.start()

    def _create_grid_world(self) -> dict:
        """
        Convert the simulation world to a grid world for planning purposes.
        """
        # each grid cell will represent 10cm
        self.grid_scale: int = 10
        self.grid_world: GridWorld = GridWorld(
            self.agent.world.field.get_length() * self.grid_scale, 
            self.agent.world.field.get_width() * self.grid_scale
        )
        logger.debug(f"[test1] grid world created with scale {self.grid_scale}")
        
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
        self.ball_grid_pos = np.array([round(ball_world_pos[0] * self.grid_scale), 
                              round(ball_world_pos[1] * self.grid_scale)])

        # convert location of agent to grid coordinates
        agent_world_pos = self.agent.world.global_position[:2] # NOTE: global_position[2] is used for detecting falls and is NOT the 2D orientation
        self.agent_grid_pos = np.array([round(agent_world_pos[0] * self.grid_scale), 
                               round(agent_world_pos[1] * self.grid_scale)])
        # TODO: add orientation information

    def _compute_ball_geometry(self):
        their_goal_pos = self.agent.world.field.get_their_goal_position()[:2]
        ball_pos = self.agent.world.ball_pos[:2]
        my_pos = self.agent.world.global_position[:2]

        ball_to_goal = their_goal_pos - ball_pos
        bg_norm = np.linalg.norm(ball_to_goal)
        if bg_norm == 0:
            return False, False, None, None, None

        ball_to_goal_dir = ball_to_goal / bg_norm

        dist = 0.30
        carry_ball_pos = ball_pos - ball_to_goal_dir * dist

        my_to_ball = ball_pos - my_pos
        my_to_ball_norm = np.linalg.norm(my_to_ball)

        if my_to_ball_norm == 0:
            my_to_ball_dir = np.zeros(2)
        else:
            my_to_ball_dir = my_to_ball / my_to_ball_norm

        cosang = np.dot(my_to_ball_dir, ball_to_goal_dir)
        cosang = np.clip(cosang, -1.0, 1.0)
        angle_diff = np.arccos(cosang)

        aligned = (my_to_ball_norm > 1e-6) and (angle_diff <= np.deg2rad(7.5))
        behind_ball = np.dot(my_pos - ball_pos, ball_to_goal_dir) < 0

        desired_orientation = MathOps.vector_angle(ball_to_goal)

        return aligned, behind_ball, carry_ball_pos, their_goal_pos, desired_orientation

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
                return (random.uniform(0, 6), random.uniform(0.5, 4), 0)
            elif self.agent.world.number == 2:
                return (random.uniform(0, 6), random.uniform(-0.5, -4), 0)
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
